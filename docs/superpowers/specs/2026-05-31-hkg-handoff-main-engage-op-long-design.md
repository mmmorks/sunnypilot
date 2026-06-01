# HKG CAN-FD Dynamic Handoff — `main` button engages openpilot longitudinal

Date: 2026-05-31
Status: Design approved, pending implementation plan
Repos touched: `sunnypilot` (main) + `opendbc_repo` submodule

## Problem

On the GV70 dynamic-radar-handoff setup, pressing the cruise **main** button engages the
car's **stock** SCC for longitudinal while openpilot only runs MADS lateral. Confirmed from
rlogs `0000037d` / `0000037e` (2026-05-31, HEAD `c96cecc8d`):

- The handoff leaves the stock ADAS-DRV/SCC ECU **alive whenever op is disengaged** (by design,
  so stock ACC/AEB works when op is off). So a `main` press engages the live stock SCC.
- `main` is **not** an op-longitudinal engage trigger today — only SET/RES (`accelCruise`/
  `decelCruise`) are (`opendbc/car/interfaces.py:388-394 update_button_enable`). `main` only
  engages MADS **lateral** (`sunnypilot/mads/mads.py:166-168`, emits `lkasEnable`).
- On the **first** engage of a drive, RES is also blocked (`resumeBlocked`: `vCruise > 250` =
  no set-speed yet, `selfdrived.py:234-237`), so only SET engages op. Result on 037d: ~25 s of
  **stock-driven** speed before the user pressed SET at t≈94.7 s. `cruiseMismatch` fired but its
  alert is commented out, so there was no feedback.

User expectation (matches stock behavior): **pressing `main` should engage openpilot fully —
lateral + longitudinal — at the current speed, even with the gas pressed.**

## Goals

1. Pressing `main` engages openpilot longitudinal (+ lateral) for cars where it makes sense
   (see Gating), seeding the set-speed from current `vEgo`.
2. Minimize the brief window where the stock SCC and op both transmit `SCC_CONTROL` at the
   engage transition (the "fight" window).

## Non-goals

- No change to LKAS-button (still lateral-only), SET/RES, or `main`-toggle-off (still disengages).
- No "track set-speed up while gas held" — set-speed is `vEgo` **at the press instant** (user
  accepted: engage at 30, gas to 50, lift → op eases back toward 30).
- No new user-facing toggle/param.
- No change for users without UEM + `MadsMainCruiseAllowed`.
- Not fixing the FCA DTC, the `resumeBlocked`-on-RES behavior, or re-enabling `cruiseMismatch`
  alerts (separate items).

## Component 1 — `main` engages op longitudinal

### Behavior
For cars where **`CP.openpilotLongitudinalControl` AND `MadsUnifiedEngagementMode` AND
`MadsMainCruiseAllowed`** are all true: the `main`-cruise press that turns cruise **available**
on (rising edge of `cruiseState.available`) engages openpilot **lateral + longitudinal** at
current speed. Works with gas pressed (op enters its overriding state — `gasPressed` raises
`gasPressedOverride`, an `OVERRIDE_LONGITUDINAL` alert with **no** `NO_ENTRY`;
`car_specific.py:141-142`). Brake still blocks (`pedalPressed` `NO_ENTRY`), matching stock.

This makes `main` honor UEM the way SET/RES already do — today `main` engages lateral-only even
under UEM.

### Implementation seam (Approach A — MADS layer)
In `sunnypilot/mads/mads.py:update_events`, the `else` branch (~165-168) already emits
`lkasEnable` on the cruise-available rising edge when `main_enabled_toggle`. Add: when also
`unified_engagement_mode` and `self.selfdrive.CP.openpilotLongitudinalControl`, emit the
op-engage event (`EventName.buttonEnable`) into `self.events`.

Why here:
- This addition runs in the `else` branch, **after** the `block_unified_engagement_mode()`
  check (which only strips engages in the `if selfdrive_enable_events:` branch). So it is not
  stripped when MADS lateral is already active — the exact failure mode that sinks the
  carstate-layer alternative (Approach B), where `main`→`CS.buttonEnable` would route through
  the `selfdrive_enable_events` path and be stripped.
- The opendbc-base alternative (Approach C: add `mainCruise` to `update_button_enable`) is too
  broad — every brand/car regardless of MADS/UEM. Rejected.

The emitted `buttonEnable` flows to the selfdrived state machine → op `enabled` →
`initialize_v_cruise` (`selfdrive/car/cruise.py:141-152`) seeds the set-speed from `vEgo`.

### Edge cases
- Already fully engaged + `main` press → toggles cruise off → disengage (existing).
- No UEM or no `MadsMainCruiseAllowed` → unchanged.
- Brake / below-engage-speed / fault at the press → op-engage blocked (correct); stock may
  engage alone in that narrow case — pre-existing behavior, not made worse.
- First-engage `resumeBlocked` dead-end is resolved because `main` now gives a working first
  engage. RES-without-prior-SET stays blocked (unchanged, fine).

## Component 2 — minimize the engage silence→op-control overlap

### Current behavior (`opendbc/car/hyundai/carcontroller.py:182-193`)
At the engage edge, op's `SCC_CONTROL` starts **immediately** (gated on `CC.enabled`, in
`create_canfd_msgs`). The stock-ECU silencing is a 2-step UDS sequence queued in `_handoff_seq`:
`extendedSession (0x10 03)` → `disableRxAndTx (0x28 03)` on `0x730`/E-CAN. The watchdog
(`_tick_handoff_watchdog`) runs **one step per cycle, waiting for each ACK**, so the stock SCC
goes silent only after **two sequential round-trips**. Measured overlap (both src on
`SCC_CONTROL 0x1a0` in a 20 ms bin): 037d 0.22 s total across 4 transitions (40–60 ms each);
037e 0.18 s across 3 (60–80 ms each). No sustained fight anywhere.

Constraints:
- ECU is in **default** session at engage; `0x10 03` must establish extended session **before**
  `0x28 03` is honored.
- panda accepts `0x28 03` (`disableRxAndTx`) **only while `controls_allowed`**, which settles a
  frame after engage.

### Approach — pipeline the two UDS steps
Send `0x10 03` and `0x28 03` back-to-back (session-control ordered first on the bus) instead of
waiting for the first ACK before sending the second, cutting roughly one round-trip (~half the
overlap). Requirements:
- The watchdog must track **two outstanding** requests and match each response by **service ID**
  (`0x50` for session control, `0x68` for communication control) rather than assuming a single
  outstanding step. NRC (`0x7F <svc> <code>`) and timeout/retry detection must be preserved
  per step.
- Op's `SCC_CONTROL` keeps starting at engage — never gap it (the car must never see zero SCC).
- `0x28 03` still only sent once `controls_allowed` permits; if that means it lands one cycle
  after `0x10 03`, that is acceptable.

This component is independently valuable (helps every engage, not just `main`).

## Safety analysis

- This change does **not** introduce a new fight mode. Fight prevention is the handoff's
  **engage-time UDS silencing**, which is empirically clean: stock+op overlap only ~40–80 ms per
  transition before the stock ECU goes silent (measured, both drives). Component 2 shrinks that
  window further.
- Component 1 **removes** a bad state: today's first engage gives ~25 s of stock-alone driving;
  after the change, op controls within one transition.
- Dependency/caveats (pre-existing, not worsened): (1) if the silencing UDS fails on engage
  (watchdog / `adasDrvHandoffEngageFail` territory), stock could persist; (2) if op-engage is
  blocked at the `main` press (brake/speed/fault) while stock engages on the button, stock drives
  alone. A hard op-independent guarantee isn't architecturally possible — the stock SCC is a live
  same-bus peer, so only the UDS silencing can remove it.

## Testing

- **Component 1** — `sunnypilot` MADS unit tests: cruise-available rising edge with
  (UEM + `MadsMainCruiseAllowed` + op-long) → op engages (`buttonEnable` emitted, state machine
  enables); without UEM → lateral-only; with gas pressed → still engages; with brake → blocked.
- **Component 2** — opendbc handoff tests (`tests/test_canfd_dynamic_handoff.py` and
  `test_hyundai.py`): pipelined engage sequence still issues `0x10 03` then `0x28 03`, watchdog
  still latches `engageFailed` on NRC/timeout of either step (service-ID matching), no regression
  in disengage sequence. Re-measure `SCC_CONTROL` dual-source overlap on a new drive; target
  below the current 40–80 ms/transition.
- Full opendbc safety/test suite green (libsafety parallel-build race is flaky — re-run).

## Open items to verify during implementation planning

1. Confirm emitting `EventName.buttonEnable` from the MADS layer is consumed by the state machine
   in the **same** cycle (ordering of MADS `update_events` vs `state_machine.update` in
   `selfdrived`).
2. Confirm `main_enabled_toggle` ↔ `MadsMainCruiseAllowed` and `unified_engagement_mode` ↔
   `MadsUnifiedEngagementMode` param mappings.
3. Confirm the watchdog rework cleanly handles two-in-flight without misattributing the existing
   1 Hz tester-present responses.

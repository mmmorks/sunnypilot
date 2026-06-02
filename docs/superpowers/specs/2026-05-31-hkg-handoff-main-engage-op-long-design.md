# HKG CAN-FD Dynamic Handoff — `main` button engages openpilot longitudinal

> **ROLLED BACK (2026-06-01).** This approach was reverted. Live drives (`0000037f`, `00000380`)
> showed that suppressing stock long on the `main` press requires claiming longitudinal authority
> (`controls_allowed`), which is only safe if op simultaneously *provides* longitudinal — i.e. full
> main-engage, with the controlsMismatch/handoff fragility that entails. We instead keep `main` =
> MADS lateral only (`lkasEnable`), op long on SET, stock SCC doing longitudinal in between.
> Retained as historical record of the tried approach.

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

### Implementation seam — `selfdrived.update_events`, before the op state machine

**Critical ordering fact** (verified): `SelfdriveD.step()` runs `update_events` →
`state_machine.update(self.events)` (op consumes events) → `mads.update()`
(`selfdrive/selfdrived/selfdrived.py:614-620`). `self.events` is cleared at the top of
`update_events` (line 199) and `self.CS_prev` is set at the end of `step()` (625). So:
- The op-engage event MUST be in `self.events` **during** `update_events` (before line 618).
- The MADS layer's `update_events` runs at 620, **after** op already consumed events — so its
  `block_unified_engagement_mode()` strip only affects MADS's own state machine, never op. An
  earlier draft placed the emit in MADS `update_events`; that cannot engage op (added too late,
  cleared next cycle). Rejected.

**Seam:** in `SelfdriveD.update_events` (`selfdrived.py`, near the existing `resumeBlocked`
block ~234-237, which already adds an op-engage-related event before the state machine and
references `self.CP`/`self.mads`), add on the `cruiseState.available` rising edge:

```python
# main-button engages openpilot longitudinal (unified) for op-long cars w/ MADS UEM + main allowed
if (self.CP.openpilotLongitudinalControl and self.mads.unified_engagement_mode
    and self.mads.main_enabled_toggle
    and CS.cruiseState.available and not self.CS_prev.cruiseState.available):
  self.events.add(EventName.buttonEnable)
```

Flow: op state machine (618) sees `buttonEnable` → `ET.ENABLE` → engages (gas raises only
`gasPressedOverride`, no `NO_ENTRY`; brake's `pedalPressed`/`preEnableStandstill` still block) →
`initialize_v_cruise` (`selfdrive/car/cruise.py:141-152`) seeds the set-speed from `vEgo`. Then
MADS at 620 sees `selfdrive_enable_events=True`; with UEM on and MADS not yet enabled it does not
strip, so the MADS state machine engages lateral on the same `buttonEnable` (unified). If MADS
lateral was already active, the strip is moot — op already engaged at 618.

Rejected alternatives: (B) set `CS.buttonEnable` in the HKG carstate — carstate lacks clean
access to the MADS UEM/`MadsMainCruiseAllowed` params and the `cruiseState.available` rising
edge; (C) add `mainCruise` to opendbc-base `update_button_enable` — too broad (every brand/car
regardless of MADS/UEM).

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

- **Component 1** — extract a testable helper `main_button_engages_op(events, *, ...)` in
  `selfdrived.py` (mirroring `mute_can_loss_at_shutdown`) and unit-test it in
  `selfdrive/selfdrived/tests/`: cruise-available rising edge with (op-long + UEM + main-allowed)
  → `buttonEnable` added / returns True; missing any gate (no UEM, no main-allowed, not op-long)
  → not added; no rising edge (already available, or unavailable) → not added.
- **Component 2** — opendbc handoff tests (`tests/test_canfd_dynamic_handoff.py` and
  `test_hyundai.py`): pipelined engage sequence still issues `0x10 03` then `0x28 03`, watchdog
  still latches `engageFailed` on NRC/timeout of either step (service-ID matching), no regression
  in disengage sequence. Re-measure `SCC_CONTROL` dual-source overlap on a new drive; target
  below the current 40–80 ms/transition.
- Full opendbc safety/test suite green (libsafety parallel-build race is flaky — re-run).

## Open items

1. ~~Same-cycle consumption from the MADS layer~~ — RESOLVED: MADS `update_events` runs *after*
   the op state machine, so the emit must live in `SelfdriveD.update_events` (see seam above).
2. CONFIRMED: `main_enabled_toggle` = `MadsMainCruiseAllowed`, `unified_engagement_mode` =
   `MadsUnifiedEngagementMode` (`sunnypilot/mads/mads.py:55-58`).
3. (Component 2, deferred) Watchdog two-in-flight + service-ID matching without misattributing
   the 1 Hz tester-present responses.

## Scope note

This plan implements **Component 1 only**. Component 2 (pipeline the engage silencing) is deferred
to a separate plan — reading `_tick_handoff_watchdog` showed its response parser is latest-frame /
single-outstanding (`carcontroller.py:240-243`), so safe pipelining needs a response-capture
rework in validated safety code for a ~10-20 ms gain on an already-clean <100 ms transition.
Revisit after Component 1 is validated on-car.

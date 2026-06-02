# HKG CAN-FD Dynamic Radar Handoff — Suppress Radar on MADS Lateral

**Date:** 2026-06-01
**Status:** Draft, pending user review
**Target scope:** HKG CAN-FD HDA II platforms already running the dynamic radar handoff (`HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF` / `hyundai_canfd_dynamic_handoff`). No change for platforms without dynamic handoff or without MADS.
**Author conversation context:** sunnypilot fork of openpilot

---

## Problem

The dynamic radar handoff silences the stock ADAS DRV ECU (`0x730`) — and thereby stock SCC/AEB radar — only while openpilot is the **longitudinal** authority. The silence edge is keyed on the longitudinal-engaged predicate at every layer:

- **panda:** `controls_allowed` (set on SET/RESUME, **not** MAIN).
- **carcontroller:** `CC.enabled` (= `selfdriveState.enabled`, **false** during MADS-lateral-only).

So when the driver presses MAIN to engage MADS lateral-only (openpilot steers, no longitudinal), the stock radar stays **active**. The desired behavior is: the stock radar is active **only when openpilot is completely disengaged** (no lateral and no longitudinal control). Any openpilot control — lateral via MAIN, or longitudinal via SET — suppresses the radar.

## Goal

Make "openpilot owns the ADAS DRV ECU role" track **either** authority (lateral or longitudinal), so the radar is suppressed whenever openpilot has any control and restored only on full disengage.

Two authority predicates already exist in the fork and are used symmetrically:

| Layer | Longitudinal authority | Lateral authority (latched) |
|---|---|---|
| panda | `controls_allowed` | `controls_allowed_lateral` (`safety/sunnypilot/mads.h`) |
| controls | `CC.enabled` | `CC_SP.mads.enabled` |

Define an "owns-the-ECU-role" predicate = **longitudinal OR latched-lateral** authority, and apply it to the *ECU-ownership* gates (silence/restore edges, ECU impersonation, forwarding). Keep the *longitudinal accel command* gated on the strict longitudinal authority, so lateral-only never commands acceleration.

## Non-goals

- **Preserving stock AEB during MADS-lateral.** Explicitly accepted by the user: while steering-only with the radar suppressed, there is no stock AEB and no openpilot longitudinal — no automatic longitudinal safety net in that mode. This is the intended tradeoff.
- **Changing steering (LKAS/LFA) behavior.** Lateral TX is already gated on `controls_allowed || controls_allowed_lateral` in `lateral.h`; unchanged here.
- **Changing the fully-disengaged behavior.** When neither lateral nor longitudinal is engaged, the ECU is restored exactly as today.
- **Any non-dynamic-handoff platform**, and **any non-MADS** configuration.

## Key design decisions

### Decision 1 — gate on the *latched* lateral state, not instantaneous `latActive`

Use `CC_SP.mads.enabled` (controls) / `controls_allowed_lateral` (panda), **not** `CC.latActive`. The latched MADS-engaged state stays true through blinker-pause and momentary steer faults. Gating the silence/restore edge on the instantaneous `latActive` would flap the radar silence↔restore (expensive UDS churn, stock AEB flickering on/off) on every blinker tap. The latched flag also mirrors panda's existing `controls_allowed_lateral`, keeping the two layers symmetric.

### Decision 2 — during MADS-lateral, impersonate the ECU exactly like upstream/vanilla

This is grounded in the true comma upstream (`commaai/opendbc` master). Upstream kills the ADAS DRV ECU at boot (`disable_ecu` in `interface.py`) and then **impersonates it at all times** — there is no engaged/disengaged gate on *sending*, only on *content*. SCC_CONTROL, ADRV broadcasts, and the 1 Hz tester-present keepalive are gated only on `openpilotLongitudinalControl`; `CC.enabled` is passed as a *parameter* that sets the active flag, not whether the message is sent:

```python
# upstream commaai/opendbc — hyundai/carcontroller.py
if self.frame % 2 == 0:                                   # SCC_CONTROL every other frame
  can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, ...))
if lka_steering:                                          # ADRV impersonation
  can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
if self.frame % 100 == 0 and ... and self.CP.openpilotLongitudinalControl:  # 1 Hz keepalive to 0x730
  can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))
```

`create_acc_control(enabled=False, …)` emits an **inactive** SCC_CONTROL (`ACCMode=0`; sunnypilot sets `aReqValue/aReqRaw` from `tuning.actual_accel`, which is 0 when disabled). Panda already accepts this with `controls_allowed` false (`longitudinal_accel_checks` permits `inactive_accel == 0`), so it is the proven-safe baseline.

What sunnypilot's dynamic handoff added is precisely the wrapper `if not (self.dynamic_radar_handoff_enabled and not CC.enabled):` around those upstream sends — it *suppresses* the impersonation in the disengaged state, because under dynamic handoff the real ECU is alive then and would double-source.

Therefore, while openpilot owns the ECU role, we drop back to the **upstream behavior**: send `create_adrv_messages` + `create_acc_control(enabled=CC.enabled, …)`. During MADS-lateral that yields an inactive SCC_CONTROL kept alive on the bus (no missing-message faults, clean lat→long transition); on SET it flips to active accel. The only dynamic-handoff difference remains the **fully-disengaged** state, where we restore the stock ECU and stop impersonating. During MADS-lateral the bus is byte-for-byte the upstream "disengaged-but-impersonating" case.

## Design — the "owns-the-ECU-role" predicate

**carcontroller:** `handoff_active = self.dynamic_radar_handoff_enabled and (CC.enabled or CC_SP.mads.enabled)`

**panda:** `controls_allowed || controls_allowed_lateral` (introduce a single local, e.g. `op_owns_adrv`, in `hyundai_canfd_tx_hook` / `hyundai_canfd_fwd_hook`).

These two are kept symmetric. Minor cross-layer timing skew on the engage edge (carcontroller sees `mads.enabled` a tick before/after panda sets `controls_allowed_lateral`) is absorbed by the existing UDS watchdog retry mechanism, exactly as it already absorbs the `controls_allowed` settle today.

## Proposed changes

### panda — `opendbc/safety/modes/hyundai_canfd.h`

1. **`handoff_blocked` (line ~187):** relax `!controls_allowed` → `!(controls_allowed || controls_allowed_lateral)`. Without this, panda would silence the radar during MADS-lateral but then *block* openpilot from impersonating the now-silent ECU's SCC_CONTROL/ADRV broadcasts, faulting the bus.
2. **`disableRxAndTx` UDS gate (line ~239):** `&& controls_allowed` → `&& (controls_allowed || controls_allowed_lateral)`. Allows the silencing frame to be sent on a MADS-lateral engage. The three restore/session frames remain allowed in any state (unchanged).
3. **`fwd_hook` (line ~287):** block stock ECU forwarding of SCC_CONTROL/ADRV when openpilot owns the role under **either** authority: `block = !hyundai_canfd_dynamic_handoff || controls_allowed || controls_allowed_lateral`.
4. **Accel-limit check (lines ~246–265): unchanged** — stays gated on `controls_allowed`. During MADS-lateral only an inactive/zero SCC_CONTROL passes; no real acceleration can be commanded.

Update the explanatory comments at each site to state the lateral-authority rationale and the retained no-accel-without-`controls_allowed` invariant.

### carcontroller — `opendbc/car/hyundai/carcontroller.py`

Compute once in `update()` (after `MadsCarController.update`, where `CC_SP` is in scope):
`handoff_active = self.dynamic_radar_handoff_enabled and (CC.enabled or CC_SP.mads.enabled)`.
Track `prev_handoff_active` and drive the handoff edges off it (the `dynamic_radar_handoff_enabled` term folds into `handoff_active`). `prev_enabled` (lines 80/138/139/205) is used only for the edges and is replaced by `prev_handoff_active`. Pass `handoff_active` down into `create_canfd_msgs` and `create_steering_messages` explicitly (instance attribute avoided — keeps the message builders testable in isolation, matching the existing direct-call tests).

1. **engage/disengage edges (lines ~138–139):** edge-detect on `handoff_active` instead of `CC.enabled`.
2. **tester-present keepalive (lines ~146–148):** keep alive while `handoff_active` (replace the `CC.enabled` term).
3. **LFA-on-E-CAN (`hyundaicanfd.create_steering_messages`, line ~57):** its `enabled` parameter's *only* use is gating the LFA duplicate on E-CAN under dynamic handoff (the real torque goes to `LKAS` on ACAN unconditionally). The carcontroller call (line ~351) must pass `handoff_active` here instead of `CC.enabled`, so OP sends LFA-on-E-CAN during MADS-lateral when the ECU is silenced. Rename the parameter to `handoff_active` for clarity (only caller is the hyundai carcontroller + tests).
4. **LFAHDA_CLUSTER (lines ~360–362):** send while `handoff_active` (the silenced ECU stops broadcasting it).
5. **ADRV impersonation (lines ~368–373):** send `create_adrv_messages` / `create_fca_warning_light` while `handoff_active`.
6. **`create_acc_control` (lines ~374–377):** send while `handoff_active`, keep passing `CC.enabled` as the active flag → inactive accel during lateral-only, active on full engage.

Item 6's `CC.enabled` argument is deliberate and is what keeps the accel inactive during MADS-lateral, matching Decision 2. Items 3–6 together are the complete set of E-CAN broadcasts openpilot impersonates while the ECU is silenced (LFA, LFAHDA_CLUSTER, ADRV 0x51/0x160/0x1ea/0x200/0x345/0x1da, SCC_CONTROL 0x1a0) — matching the panda `HYUNDAI_CANFD_LKA_STEER_MSG_LONG_HANDOFF_TX_MSGS` allow-list.

### Tests

- **`opendbc/safety/tests/test_hyundai_canfd.py`:** assert the `disableRxAndTx` silencing frame is accepted when `controls_allowed_lateral` is set (and `controls_allowed` false); assert SCC_CONTROL (inactive) and ADRV impersonation are accepted under `controls_allowed_lateral`; assert a real (non-zero) accel SCC_CONTROL is still rejected when only `controls_allowed_lateral` (no `controls_allowed`).
- **carcontroller handoff tests:** add the MADS-lateral engage edge — silence on lateral engage, ADRV + inactive SCC_CONTROL emitted during lateral-only, restore only on full disengage, no double silence on lat→lat+long transition.

## Consequence to confirm during planning

The handoff **engage-fault** path (`adasDrvHandoffEngageFail` → immediate disable) will now also fire on a MADS-lateral engage if the silencing UDS sequence fails. In that case stock AEB stays alive (safe), but openpilot would immediate-disable. Confirm during planning that the event disables the appropriate state (lateral and/or longitudinal) and that disabling lateral on a silence failure is the desired conservative behavior.

## Known limitation — brake-pause predicate divergence

The carcontroller's `handoff_active` uses the latched `CC_SP.mads.enabled` (stays True through a pause, by design — see Decision 1, to avoid UDS silence/restore flapping). Panda, however, drops `controls_allowed_lateral` on brake when `MADS_PAUSE_LATERAL_ON_BRAKE` is enabled. So during a brake-hold in PAUSE-on-brake mode under MADS-lateral, the two predicates diverge: the carcontroller keeps the ECU silenced and keeps impersonating, while panda's `handoff_blocked` blocks openpilot's ADRV/SCC_CONTROL TX (neither authority held). The result is a transient E-CAN gap where neither openpilot's ADRV/SCC nor the silenced stock ECU broadcasts.

This is accepted as non-blocking: it is transient, latches no fault (the silence/restore watchdog acts only on edges, and no edge fires during a pause), and **LFA-on-E-CAN keeps flowing** (it is not in panda's blocked ADRV list and passes the zero-torque steer check), so the counter-collision lane-keep DTC is unaffected. It also falls inside the already-accepted "stock AEB off during MADS-lateral" envelope. A future tightening could give panda a latched MADS-engaged signal for the ECU-ownership gates so it matches the carcontroller through a pause; out of scope here.

## Acceptance criteria

1. Pressing MAIN (MADS lateral engage) silences the stock ADAS DRV ECU via the existing UDS sequence; the radar/SCC/AEB go offline.
2. The stock ECU is restored only when openpilot is fully disengaged (no lateral and no longitudinal).
3. No silence↔restore flapping across blinker-pause / momentary steer-fault while MADS stays latched-engaged.
4. During MADS-lateral the bus carries openpilot's full ECU impersonation — LFA-on-E-CAN, LFAHDA_CLUSTER, ADRV, and inactive SCC_CONTROL — identical to the upstream disengaged-but-impersonating case; on SET, SCC_CONTROL becomes active with real accel; no missing-message faults across lat→long.
5. panda rejects any non-inactive accel command while only `controls_allowed_lateral` is set.
6. panda and carcontroller safety/handoff tests updated and passing.

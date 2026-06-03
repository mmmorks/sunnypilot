# HKG CAN-FD Dynamic Radar Handoff — Eliminate First-Engage LFA Counter Collision

**Date:** 2026-06-02
**Status:** Draft, pending user review
**Target scope:** HKG CAN-FD HDA II platforms running the dynamic radar handoff (`HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF`) **with `CANFD_LKA_STEER_MSG`** (LFA on E-CAN, LKAS on A-CAN — e.g. `GENESIS_GV70_ELECTRIFIED_1ST_GEN`). No change for non-dynamic-handoff platforms or non-LKA_STEER_MSG platforms.
**Author conversation context:** sunnypilot fork of openpilot

---

## Problem

On the **first openpilot engage of a drive** (typically MADS lateral on the MAIN press), the driver sees "Steering Assist Temporarily Unavailable" — escalating to the full-screen "TAKE CONTROL IMMEDIATELY" when hands have been off the wheel >1.5s. Lateral drops out for ~0.15–0.5s, then re-engages on its own.

### Measured root cause (drives 2026-06-02, master `14d7585a59`, GV70 EV)

`carState.steerFaultTemporary` is `MDPS_LkaFailSta != 0` (`carstate.py:268`). The MDPS raises it because of a **dual-sender CAN counter collision** on the `LFA` message (addr **298**, sender `ADRV`) on E-CAN:

- While **disengaged**, the dynamic handoff keeps the stock ADAS DRV ECU **live**, broadcasting `LFA(298)` at 100 Hz (its own rolling `COUNTER`).
- At engage, `handoff_active` flips True instantly, so openpilot **opens its own `LFA(298)` E-CAN gate** (`hyundaicanfd.create_steering_messages`) and begins transmitting `LFA(298)` at 100 Hz with an **independent** counter.
- openpilot's engage sequence fires `disableRxAndTx` to silence the stock ECU, but **panda rejects that frame until `controls_allowed` settles** (a few frames after engage). The stock ECU therefore keeps broadcasting `LFA(298)` until the silence finally lands.
- For the resulting window, **two senders** emit `LFA(298)` with two interleaved counter sequences → the MDPS flags `LkaFailSta` → `controlsd.py:114` drops `CC.latActive` (`... and not steerFaultTemporary`) → `car_specific.py:161-172` raises `steerTempUnavailableSilent` (hands-on <1.5s) or `steerTempUnavailable` (the loud variant).

**Evidence (`.drivedata/dual_lfa_test.py`):**
- Route 00000381: engage 193.45; `LFA(298)` dual-broadcast (E-CAN `bus1` = stock ADRV **and** `bus129` = op echo, both 100 Hz) 193.5→194.0; first `disableRxAndTx` 193.45 got **no response** (panda-blocked), re-sent 193.98, ADRV ack `02 68 03` at 194.00 → ADRV LFA stops; `LkaFailSta` onset 193.983.
- Route 00000382: engage 222.40; dual-broadcast 222.4→222.9; ack at 222.90; `LkaFailSta` onset **222.498** — early in the window, ~0.4s **before** the silence.

The fault appears **throughout** the dual-broadcast window and clears only once the stock ECU is silenced (single sender restored). The within-window onset varies with counter phase (early in 382, late in 381). This rules out a hand-over-transition cause and confirms **sustained dual-broadcast** as the trigger. It occurs only on the **first** engage because that is the only engage where the stock ECU is freshly live (broadcasting `LFA`) at the moment op engages **and** the silence frame is rejected until `controls_allowed` settles. A later in-drive engage with main-cruise already up (e.g. SET at t=234 in 381) does **not** fault.

This is **fork-specific.** True comma/sunnypilot upstream silences the ADRV ECU at **boot**, so there is never a second `LFA` sender — the collision window cannot exist. The fork's "ECU live while disengaged" design (radar/AEB active when fully disengaged) is what introduces it.

### Why masking is not a fix

During the fault `MDPS_LkaFailSta` is the MDPS **refusing** LKA torque. Suppressing op's `latActive` drop would hide the alert but the wheel still would not steer — and would hide a genuine unavailability. The fix must **prevent the collision**, not mask it.

## Goal

openpilot must never become the **second** `LFA(298)` sender. On engage it withholds its own `LFA(298)` — and its lateral actuation — until it has **confirmed** the stock ADRV ECU is silenced, then takes over as the **sole** sender with a **continuous** counter so the MDPS sees one unbroken stream across the hand-over.

## Non-goals

- **Changing the disengage path.** On disengage op already stops `LFA(298)` and the stock ECU resumes — a gap, not an overlap, so no collision. Unchanged except resetting the new latched state.
- **Changing panda / safety.** No `controls_allowed` semantics change. The fix is entirely in the fork's opendbc carcontroller/carstate/hyundaicanfd.
- **Changing the LKAS (A-CAN) torque path.** `LKAS`/`LKAS_ALT` on A-CAN has a single sender (op) and does not collide; its counter is untouched.
- **Non-dynamic-handoff or non-LKA_STEER_MSG platforms.**

## Key design decisions

### Decision 1 — gate op's LFA takeover on *confirmed* silence, not on `handoff_active`

The `LFA(298)` E-CAN send and the lateral actuation are gated on a new latched `steer_takeover_ok`, set only when op observes the `disableRxAndTx` positive ack (`0x68 0x03` on `0x738`) — i.e. proof the stock ECU has actually stopped broadcasting `LFA`. Gating on the instantaneous `handoff_active` (today's behavior) is exactly what opens the collision window, because `handoff_active` leads the silence by the panda-settle delay.

### Decision 2 — accept a brief lateral-inactive gap, but shrink it at the source

While `steer_takeover_ok` is False, op holds `apply_steer_req = 0` / `apply_torque = 0` and sends no `LFA(298)`. The stock ECU is the sole `LFA` sender during this window (its stock LFA is inactive), so lateral is genuinely inactive for the gap — there is no way to actuate before the ECU is silenced. The gap equals the silence-handshake latency.

That latency today is **~500ms, and it is self-inflicted, not protocol latency.** The stock ECU acks `disableRxAndTx` in ~20ms once it receives the frame (193.98→194.00; 222.88→222.90). The ~500ms is the watchdog's per-step ack window (`HANDOFF_RESPONSE_DEADLINE_FRAMES = 50` @ 100 Hz): the first frame is panda-dropped, and the watchdog waits the full 50 frames before retrying, even though `controls_allowed` settles within a few frames. Shrinking the per-step window to ~8 frames (80ms) and raising `HANDOFF_STEP_MAX_RETRIES` to keep the total fault-latch budget unchanged makes the silence land within ~1–2 frames of `controls_allowed` settling → gap **~50–150ms** (imperceptible). 8 frames still safely exceeds the <50ms (≤5 frame) UDS response time, so a genuine ack is never missed.

### Decision 3 — continue the LFA counter across the hand-over

When op takes over, it seeds its `LFA(298)` `COUNTER` from the **last value it observed from the stock ADRV ECU** and continues incrementing (mod 256), instead of starting a fresh sequence. With the now-tiny (~1 frame) hand-over gap, the MDPS sees one continuous counter stream (stock `…N`, then op `…N+1, N+2`) and never sees a discontinuity, eliminating any residual re-lock blip when the source switches. `CHECKSUM` is left to the packer (CRC16), matching the existing `create_suppress_lfa` pattern that overrides `COUNTER` only.

### Decision 4 — bounded fallback so a dead ECU can't strand lateral

`steer_takeover_ok = adrv_silenced or silence_timeout`. If the silence never acks within the (now larger) retry budget, `silence_timeout` opens the gate anyway — op reverts to today's behavior (transient collision/fault, but lateral works) rather than withholding steering indefinitely. This reuses the existing handoff watchdog fault-latch, which already surfaces a failed engage sequence.

## Design — data flow

```
engage_edge (handoff_active rising)
  → start _engage_handoff_seq (existing): extendedSession, then disableRxAndTx
  → adrv_silenced = False; start silence deadline
  → each frame while handoff_active and not steer_takeover_ok:
        withhold LFA(298); apply_steer_req = apply_torque = 0   (lateral held)
  → watchdog observes 0x68 0x03 ack  → adrv_silenced = True
        (faster retry window lands this within ~1-2 frames of controls_allowed settling)
  → steer_takeover_ok True:
        seed self.lfa_counter from last-observed stock ADRV LFA COUNTER
        send LFA(298) (COUNTER continues stock's), resume normal apply_torque
  → fallback: silence deadline exceeded → silence_timeout True → gate opens anyway
disengage_edge (handoff_active falling)
  → _disengage_handoff_seq (existing) re-enables ECU
  → adrv_silenced = False; reset silence_timeout
```

## Proposed changes

### `opendbc/car/hyundai/carstate.py`

Snapshot the **stock ADRV `LFA(298)` `COUNTER`** so the carcontroller can continue it (mirrors the existing `adas_drv_uds_response_*` snapshot, gated on `CANFD_DYNAMIC_HANDOFF`):
1. Subscribe to `LFA` (`COUNTER`) on the E-CAN parser when `CANFD_DYNAMIC_HANDOFF` is set (sporadic-safe `float('nan')` freq, matching the `ADAS_DRV_UDS_RESPONSE` subscription pattern).
2. Each `update_canfd`, store the latest received `LFA` `COUNTER` into a new field (e.g. `self.adrv_lfa_counter`). Only valid/meaningful while the stock ECU is live (disengaged / pre-silence), which is exactly when it is needed for the seed.

### `opendbc/car/hyundai/carcontroller.py`

1. **New latched state** (in `__init__`): `self.adrv_silenced = False`, `self.silence_timeout = False`, a silence-deadline frame stamp, and `self.lfa_counter` (running LFA counter once op owns the stream).
2. **Watchdog (`_tick_handoff_watchdog`):** when the active engage sequence's `disableRxAndTx` step gets its `0x68 0x03` positive ack, set `self.adrv_silenced = True`. Set `self.silence_timeout = True` if the engage silence step exhausts its retry budget.
3. **Edges (lines ~144-145):** on `disengage_edge`, reset `adrv_silenced`, `silence_timeout`, and the deadline. On `engage_edge`, arm the deadline.
4. **Faster handshake (Decision 2):** reduce `HANDOFF_RESPONSE_DEADLINE_FRAMES` 50 → ~8 and raise `HANDOFF_STEP_MAX_RETRIES` so the total per-step timeout budget (`deadline × (retries+1)`) stays ≈ today's (~2s). Update the explanatory comment to state the rationale (first silence frame is panda-dropped until `controls_allowed` settles; fast retry catches the settle).
5. **Steer-takeover gate:** compute `steer_takeover_ok = self.adrv_silenced or self.silence_timeout`. When `handoff_active and not steer_takeover_ok`, force `apply_steer_req = 0` and `apply_torque = 0` (extends the existing `if not CC.latActive: apply_torque = 0` block at lines ~130-131).
6. **LFA counter (Decision 3):** at the takeover frame, seed `self.lfa_counter = (CS.adrv_lfa_counter + 1) & 0xFF`; thereafter increment mod 256 each frame op emits `LFA`. Pass `steer_takeover_ok` and `self.lfa_counter` into `create_canfd_msgs` → `create_steering_messages`.

### `opendbc/car/hyundai/hyundaicanfd.py` — `create_steering_messages`

- Rename the `handoff_active` parameter to `lfa_send_ok` (its sole use is gating the E-CAN `LFA` duplicate); pass `steer_takeover_ok` from the carcontroller. The `LKAS`/`LKAS_ALT` A-CAN send stays unconditional.
- Add an `lfa_counter` parameter; when sending `LFA`, build the values with an explicit `COUNTER` (`{**values, "COUNTER": lfa_counter}`) so the packer uses the continued counter and computes `CHECKSUM`. The A-CAN `LKAS` message keeps the packer's auto-incremented counter (single sender, no collision).

### Tests — `opendbc` carcontroller / hyundaicanfd direct-call tests

1. **No second sender during handshake:** on engage, assert no `LFA(298)` is emitted and `apply_steer_req/apply_torque == 0` until a synthesized `0x68 0x03` ack is fed; then assert `LFA(298)` resumes and torque is applied.
2. **Counter continuity:** feed a stock `adrv_lfa_counter = N`; after takeover, assert op's first `LFA(298)` `COUNTER == (N+1) & 0xFF` and that it increments mod 256 thereafter.
3. **Faster retry:** with the new window, assert the silence step re-sends within ~8 frames and succeeds on a delayed-accept (first attempt dropped, second acked); assert total timeout budget ≈ unchanged.
4. **Timeout fallback:** with no ack, assert `silence_timeout` opens the gate (LFA + torque resume) within the retry budget and the existing handoff fault latches.
5. **Disengage reset:** assert `adrv_silenced`/`silence_timeout` reset on `disengage_edge`.

### On-car verification

First MAIN engage of a drive: no "Steering Assist Temporarily Unavailable" / "TAKE CONTROL IMMEDIATELY"; lateral engages within ~100–150ms; `carState.steerFaultTemporary` stays False through the engage; replay of a new drive shows single-sender `LFA(298)` (no `bus1`+`bus129` overlap) across the engage edge.

## Consequence to confirm during planning

- **Lateral held while `handoff_active` but actuation suppressed:** during the ~50–150ms handshake `CC.latActive` may be True (op's view) while the carcontroller holds torque at 0. Confirm this does not trip the `>90°`/`common_fault_avoidance` counters or any `latActive`-vs-output consistency check, and that the UI's brief "engaged but not yet steering" state is acceptable.
- **`adrv_lfa_counter` validity at seed time:** confirm the last stock counter observed just before silence is the correct seed (off-by-one tolerance of the MDPS counter check), and that a missed/stale snapshot falls back gracefully (e.g. to the packer's auto-counter) rather than seeding a stale value.

## Acceptance criteria

1. First-engage `LFA(298)` is single-sender: op emits no `LFA(298)` until the stock ECU's `disableRxAndTx` ack is observed; replay shows no E-CAN `bus1`+`bus129` overlap across the engage edge.
2. `carState.steerFaultTemporary` stays False through the first engage; neither `steerTempUnavailable` variant fires.
3. Lateral-inactive gap at first engage ≤ ~150ms (down from ~500ms) via the shortened retry window; total handoff fault-latch budget unchanged.
4. op's `LFA(298)` `COUNTER` continues from the last stock value (+1) across the hand-over; no counter discontinuity.
5. A silence that never acks opens the gate within the retry budget (lateral still works) and latches the existing handoff fault.
6. Disengage behavior unchanged; latched state resets on disengage.
7. opendbc carcontroller/hyundaicanfd tests updated and passing.

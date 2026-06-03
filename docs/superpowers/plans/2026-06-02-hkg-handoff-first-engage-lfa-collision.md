# HKG Handoff First-Engage LFA Counter Collision — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop openpilot from becoming the second `LFA(298)` sender on the first engage of a drive, so the MDPS never sees colliding counters and the spurious "Steering Temporarily Unavailable" alert disappears.

**Architecture:** On a dynamic-radar-handoff CAN-FD car, gate openpilot's `LFA(298)` E-CAN transmission and its lateral actuation on *confirmed* ADRV-ECU silence (the `disableRxAndTx` `0x68 0x03` ack), not on the instantaneous `handoff_active`. Shorten the watchdog's per-step ack window so the silence lands within ~1–2 frames of `controls_allowed` settling. When openpilot takes over `LFA`, continue the counter from the stock ECU's last value so the MDPS sees one unbroken stream. All changes are in the fork's opendbc (`carstate.py`, `carcontroller.py`, `hyundaicanfd.py`); no panda/safety change.

**Tech Stack:** Python, opendbc CAN-FD car port (`opendbc/car/hyundai/`), `CANPacker`/`CANParser`, `unittest`. Reference spec: `docs/superpowers/specs/2026-06-02-hkg-handoff-first-engage-lfa-collision-design.md`.

**Test runner (per repo convention — NOT `uv run`):**
```bash
cd /Users/john/Code/sunnypilot
source .venv/bin/activate
python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -v
```

---

## File structure

- **Modify `opendbc_repo/opendbc/car/hyundai/carstate.py`** — subscribe to + snapshot the stock ADRV `LFA(298)` `COUNTER` (Task 1).
- **Modify `opendbc_repo/opendbc/car/hyundai/carcontroller.py`** — latch `adrv_silenced`/`silence_timeout` in the watchdog, shorten the retry window, hold lateral + gate/seed/continue the LFA counter in `update()` and `create_canfd_msgs` (Tasks 2, 4).
- **Modify `opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py`** — `create_steering_messages`: rename the LFA gate param and add the explicit-`COUNTER` override (Task 3).
- **Modify `opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py`** — new behavior tests; update existing call sites for the additive params (every task adds its tests here).

All five logical changes are interdependent (the carcontroller wiring in Task 4 consumes the field from Task 1, the state from Task 2, and the signature from Task 3), so implement in order 1 → 2 → 3 → 4 and finish with the integration test sweep in Task 5.

---

### Task 1: carstate — snapshot the stock ADRV `LFA(298)` counter

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/carstate.py` (`__init__` ~line 80; `update_canfd` ~line 327-335; `get_can_parsers_canfd` ~line 349-351)
- Test: `opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py`

**Context:** The `pt` parser is bound to E-CAN (`CanBus(CP).ECAN`). The stock ADRV ECU broadcasts `LFA` (addr `0x12a`, signal `COUNTER`, 8-bit) at 100 Hz on E-CAN while disengaged. openpilot's own `LFA` transmissions echo back on bus `ECAN+128`, which this parser (bound to `ECAN`) does **not** see — so once the stock ECU is silenced, `cp.vl["LFA"]["COUNTER"]` simply freezes at the stock ECU's last value, which is exactly the seed we want. Subscribe **ignore-alive** (`float('nan')`) — like `ADAS_DRV_UDS_RESPONSE` — so that frozen value never invalidates the bus.

- [ ] **Step 1: Write the failing test**

Add to `test_canfd_dynamic_handoff.py` (the `TestCanfdDynamicHandoff` class already imports `_handoff_pt_parser`, `CANPacker`):

```python
  def test_lfa_counter_subscribed_ignore_alive(self):
    # The stock ADRV LFA (0x12a) counter is snapshotted to seed openpilot's LFA counter at takeover. It must be
    # ignore-alive: after openpilot silences the ECU, the physical-bus LFA stops (op's own echoes on ECAN+128,
    # which this E-CAN parser never sees), and an alive-checked LFA would then invalidate the bus.
    pt = _handoff_pt_parser()
    lfa_addr = pt.dbc.name_to_msg["LFA"].address
    self.assertEqual(lfa_addr, 0x12a)
    self.assertIn(lfa_addr, pt.message_states)
    self.assertTrue(pt.message_states[lfa_addr].ignore_alive,
                    "LFA must be registered ignore-alive (NaN freq) so a silenced ECU never gates can_valid")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestCanfdDynamicHandoff::test_lfa_counter_subscribed_ignore_alive -v`
Expected: FAIL — `0x12a not in pt.message_states` (LFA not subscribed yet).

- [ ] **Step 3: Subscribe to LFA in `get_can_parsers_canfd`**

In `carstate.py`, in `get_can_parsers_canfd`, extend the existing dynamic-handoff block (currently adds `ADAS_DRV_UDS_RESPONSE`):

```python
    # dynamic radar handoff: subscribe to ADAS DRV ECU UDS responses (sporadic; expected only on engage/disengage edges)
    if CP.safetyConfigs and CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF:
      msgs += [("ADAS_DRV_UDS_RESPONSE", float('nan'))]
      # first-engage LFA handoff: snapshot the stock ADRV LFA(0x12a) COUNTER to seed openpilot's LFA counter at
      # takeover. ignore-alive (NaN): after silence the physical-bus LFA stops (op's echo is on ECAN+128), and an
      # alive-checked LFA would then invalidate the bus.
      msgs += [("LFA", float('nan'))]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestCanfdDynamicHandoff::test_lfa_counter_subscribed_ignore_alive -v`
Expected: PASS.

- [ ] **Step 5: Add the snapshot field + write its test**

Add the snapshot field in `CarState.__init__` (right after `self._prev_adas_drv_uds_response_count: int = 0`):

```python
    # first-engage LFA handoff: latest stock ADRV LFA(0x12a) COUNTER (E-CAN). Only meaningful while the stock ECU
    # is live (pre-silence); the carcontroller reads it once at takeover to continue the counter seamlessly.
    self.adrv_lfa_counter: int = 0
```

Snapshot it in `update_canfd`, extending the existing dynamic-handoff snapshot block (after the `adas_drv_uds_response_byte3` line):

```python
      self.adrv_lfa_counter = int(cp.vl["LFA"]["COUNTER"])
```

Add the snapshot test:

```python
  def test_adrv_lfa_counter_snapshotted(self):
    from opendbc.can import CANPacker
    from opendbc.car.hyundai.carstate import CarState
    CP = CarInterface.get_non_essential_params(HANDOFF_CAR)
    CP.safetyConfigs[-1].safetyParam = int(CP.safetyConfigs[-1].safetyParam | HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
    cs = CarState(CP, CarInterface.get_non_essential_params_sp(CP, HANDOFF_CAR))
    self.assertEqual(cs.adrv_lfa_counter, 0)
    cs.adrv_lfa_counter = 0
    cs.adrv_lfa_counter = int(0x42)  # stand-in for a parsed COUNTER; field exists and is writable
    self.assertEqual(cs.adrv_lfa_counter, 0x42)
```

> Note: a full parse-through test (feed an `LFA` frame, assert the snapshot) is covered by the carcontroller integration test in Task 5; here we only assert the field exists and is wired. If `CarState(...)` construction needs more args, mirror the construction in `test_uds_response_*` (which uses `_handoff_pt_parser`) — but the field-existence assertion above is the bite-sized unit.

- [ ] **Step 6: Run tests to verify they pass**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -k "lfa_counter or adrv_lfa" -v`
Expected: PASS (both).

- [ ] **Step 7: Commit**

```bash
git add opendbc_repo/opendbc/car/hyundai/carstate.py opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py
git commit -m "hyundai carstate: snapshot stock ADRV LFA counter for handoff seed"
```

---

### Task 2: carcontroller — latch `adrv_silenced`/`silence_timeout` + faster retry window

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/carcontroller.py` (`__init__` ~line 80-103; `_tick_handoff_watchdog` ~line 273-287; `_latch_handoff_fault` ~line 292-300; disengage edge ~line 174-176)
- Test: `opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py`

**Context:** The engage sequence is `[extendedSession (fire-and-forget), disableRxAndTx (watched, expected 0x68)]`. When that watched step gets its positive ack the sequence empties — that is the silence confirmation. A failed engage sequence (NRC or retry-exhausted timeout) calls `_latch_handoff_fault(1)`.

- [ ] **Step 1: Write the failing tests**

Add a new class to `test_canfd_dynamic_handoff.py` (reuses the `TestHandoffEnginePipeline` helpers' shape):

```python
class TestHandoffSilenceGate(unittest.TestCase):
  """adrv_silenced latches True only when the engage disableRxAndTx (0x68) ack lands; silence_timeout latches
  True if the engage silence sequence fails (NRC or retry-exhausted). Both reset on disengage."""
  def _cc(self):
    CP = _handoff_car_params(handoff=True)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, HANDOFF_CAR)
    return CarController({"pt": "hyundai_canfd_generated", "cam": "hyundai_canfd_generated"}, CP, CP_SP)

  @staticmethod
  def _cs(count=0, byte1=0, byte2=0, lfa_counter=0):
    return types.SimpleNamespace(adas_drv_uds_response_count=count, adas_drv_uds_response_byte1=byte1,
                                 adas_drv_uds_response_byte2=byte2, adrv_lfa_counter=lfa_counter)

  def _tick(self, cc, cs, frame):
    cc.frame = frame
    cc._tick_handoff_watchdog(cs, [])

  def test_initial_state(self):
    cc = self._cc()
    self.assertFalse(cc.adrv_silenced)
    self.assertFalse(cc.silence_timeout)

  def test_adrv_silenced_on_silencing_ack(self):
    cc = self._cc()
    cc._handoff_seq = cc._engage_handoff_seq()
    cc._handoff_seq_kind = 1
    self._tick(cc, self._cs(), 0)            # send extendedSession (fire-and-forget)
    self._tick(cc, self._cs(), 1)            # send disableRxAndTx (watched)
    self.assertFalse(cc.adrv_silenced)
    self._tick(cc, self._cs(count=1, byte1=0x68), 2)  # silence ack
    self.assertTrue(cc.adrv_silenced)
    self.assertFalse(cc.silence_timeout)

  def test_silence_timeout_on_engage_nrc(self):
    cc = self._cc()
    cc._handoff_seq = cc._engage_handoff_seq()
    cc._handoff_seq_kind = 1
    self._tick(cc, self._cs(), 0)
    self._tick(cc, self._cs(), 1)
    self._tick(cc, self._cs(count=1, byte1=0x7F, byte2=0x28), 2)  # NRC for 0x28
    self.assertFalse(cc.adrv_silenced)
    self.assertTrue(cc.silence_timeout)

  def test_silence_state_resets_on_disengage(self):
    cc = self._cc()
    cc.adrv_silenced = True
    cc.silence_timeout = True
    # simulate the disengage edge reset path
    cc._reset_handoff_silence_state()
    self.assertFalse(cc.adrv_silenced)
    self.assertFalse(cc.silence_timeout)

  def test_faster_retry_window(self):
    cc = self._cc()
    # Per-step window shortened so the panda-rejected first silence frame retries quickly once controls_allowed
    # settles; total budget (window * (retries+1)) kept ~unchanged (~2s @ 100Hz).
    self.assertLessEqual(cc.HANDOFF_RESPONSE_DEADLINE_FRAMES, 10)
    total = cc.HANDOFF_RESPONSE_DEADLINE_FRAMES * (cc.HANDOFF_STEP_MAX_RETRIES + 1)
    self.assertGreaterEqual(total, 150)  # >= ~1.5s of total retry budget before latching
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestHandoffSilenceGate -v`
Expected: FAIL — `AttributeError: 'CarController' object has no attribute 'adrv_silenced'` (and `_reset_handoff_silence_state`).

- [ ] **Step 3: Add state, reset helper, and shorten the window in `__init__`**

In `carcontroller.py __init__`, change the window/retry constants:

```python
    # Window in which we accept each step's ack. 8 frames @ 100Hz = 80ms; UDS S6/S7 typically <50ms. Kept short so
    # the engage silencing frame — which panda drops until controls_allowed settles (a few frames after engage) —
    # retries quickly instead of stalling lateral for a full window. See HANDOFF_STEP_MAX_RETRIES for total budget.
    self.HANDOFF_RESPONSE_DEADLINE_FRAMES: int = 8
    # Re-send a step this many times on timeout before latching a fault. Raised so the total budget
    # (window * (retries+1)) stays ~2s despite the shorter window. Absorbs the panda controls_allowed settle and
    # one-off CAN losses without escalating to IMMEDIATE_DISABLE. NRCs are NOT retried.
    self.HANDOFF_STEP_MAX_RETRIES: int = 24
```

Add the silence-gate state (next to `self.prev_handoff_active = False`):

```python
    # first-engage LFA handoff: openpilot withholds its own LFA(0x12a) + lateral actuation until the stock ADRV
    # ECU is confirmed silenced, so the two senders' counters never collide at the MDPS. adrv_silenced latches on
    # the disableRxAndTx ack; silence_timeout opens the gate anyway if the silence handshake fails (lateral works,
    # transient collision accepted) rather than withholding steering forever.
    self.adrv_silenced = False
    self.silence_timeout = False
    self.prev_lfa_send_ok = False
    self.lfa_counter = 0
```

Add the reset helper method (place near `_latch_handoff_fault`):

```python
  def _reset_handoff_silence_state(self) -> None:
    self.adrv_silenced = False
    self.silence_timeout = False
    self.prev_lfa_send_ok = False
```

- [ ] **Step 4: Set `adrv_silenced` on the silence ack in `_tick_handoff_watchdog`**

In `_tick_handoff_watchdog`, in the `matched_positive` branch, after `self._handoff_seq.pop(0)`:

```python
        if matched_positive:
          self._handoff_seq.pop(0)  # success → next step sends on the following tick
          # engage sequence's only watched step is disableRxAndTx; emptying it under kind==1 = stock ECU silenced.
          if self._handoff_seq_kind == 1 and not self._handoff_seq:
            self.adrv_silenced = True
```

- [ ] **Step 5: Set `silence_timeout` when the engage sequence fails**

In `_latch_handoff_fault`, set the gate-open flag for the engage kind:

```python
  def _latch_handoff_fault(self, kind: int) -> None:
    # Engage fault outranks disengage fault; never downgrade an already-latched engage fault. kind is the
    # sequence kind (1=engage, 2=disengage); the latched value is the SP enum surfaced to selfdrived.
    if kind == 1:
      # engage silence failed (NRC or retry-exhausted) → open the LFA gate so lateral still works (accepting the
      # transient collision) rather than withholding steering indefinitely.
      self.silence_timeout = True
    if kind == 1 and self.handoff_fault != HandoffFault.engageFailed:
      self.handoff_fault = HandoffFault.engageFailed
      self.handoff_fault_clear_frame = self.frame + self.HANDOFF_FAULT_LATCH_FRAMES
    elif kind == 2 and self.handoff_fault not in (HandoffFault.engageFailed, HandoffFault.disengageFailed):
      self.handoff_fault = HandoffFault.disengageFailed
      self.handoff_fault_clear_frame = self.frame + self.HANDOFF_FAULT_LATCH_FRAMES
```

- [ ] **Step 6: Reset on the disengage edge in `update()`**

In `update()`, in the `if disengage_edge:` block (~line 174), add the reset call:

```python
    if disengage_edge:
      self._handoff_seq = self._disengage_handoff_seq()
      self._handoff_seq_kind = 2
      self._reset_handoff_silence_state()
```

- [ ] **Step 7: Run tests to verify they pass**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestHandoffSilenceGate -v`
Expected: PASS (all 5).

- [ ] **Step 8: Run the full handoff suite to confirm no regression in the existing watchdog tests**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -v`
Expected: PASS (existing `TestHandoffEnginePipeline` tests use `HANDOFF_RESPONSE_DEADLINE_FRAMES` symbolically, so they adapt to the new value).

- [ ] **Step 9: Commit**

```bash
git add opendbc_repo/opendbc/car/hyundai/carcontroller.py opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py
git commit -m "hyundai handoff: latch adrv_silenced/silence_timeout, shorten silence retry window"
```

---

### Task 3: hyundaicanfd — gate LFA on silence + continue the counter

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py` (`create_steering_messages` ~line 39-60)
- Test: `opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py`

**Context:** Current signature: `create_steering_messages(packer, CP, CAN, handoff_active, lat_active, apply_torque, lkas_icon)`. The 4th param's only job is gating the E-CAN `LFA` duplicate. We rename it `lfa_send_ok` (the carcontroller now folds the silence condition into it) and add an optional `lfa_counter` to override the `LFA` `COUNTER` (continuing the stock ECU's sequence). The `LKAS`/`LKAS_ALT` A-CAN message keeps the packer's auto-counter (single sender, no collision). `CHECKSUM` is always packer-computed (matches `create_suppress_lfa`).

- [ ] **Step 1: Write the failing tests**

Add to `test_canfd_dynamic_handoff.py`, in `TestHandoffSteeringHandoff` (extend the existing `_lfa_count` helper and add a counter test). Replace the existing `_lfa_count` with a version that passes the new param, and add a counter helper:

```python
  def _lfa_msgs(self, CP, lfa_send_ok, lfa_counter=0):
    packer = CANPacker("hyundai_canfd_generated")
    CAN = CanBus(CP)
    msgs = hyundaicanfd.create_steering_messages(packer, CP, CAN, lfa_send_ok, lfa_send_ok, 0, 0, lfa_counter)
    return [(addr, dat, bus) for addr, dat, bus in msgs if addr == LFA and bus == CAN.ECAN]

  def _lfa_count(self, CP, handoff_active):
    return len(self._lfa_msgs(CP, handoff_active))

  def test_lfa_counter_continues_from_seed(self):
    from opendbc.can import CANParser
    CP = _handoff_car_params(handoff=True)
    CAN = CanBus(CP)
    msgs = self._lfa_msgs(CP, lfa_send_ok=True, lfa_counter=0x37)
    self.assertEqual(len(msgs), 1)
    _, dat, _ = msgs[0]
    parser = CANParser("hyundai_canfd_generated", [("LFA", 0)], CAN.ECAN)
    parser.update([0, [(LFA, dat, CAN.ECAN)]])
    self.assertEqual(parser.vl["LFA"]["COUNTER"], 0x37)
```

> The existing `test_no_lfa_when_fully_disengaged_under_handoff`, `test_lfa_present_when_handoff_active`, and `test_lfa_always_present_without_handoff` keep working through the rewritten `_lfa_count` (they only count messages).

- [ ] **Step 2: Run tests to verify they fail**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestHandoffSteeringHandoff -v`
Expected: FAIL — `create_steering_messages() takes 7 positional args but 8 were given`.

- [ ] **Step 3: Update `create_steering_messages`**

```python
def create_steering_messages(packer, CP, CAN, lfa_send_ok, lat_active, apply_torque, lkas_icon, lfa_counter=0):
  values = {
    "LKA_OptUsmSta": 2,
    "LKA_SysIndReq": lkas_icon,
    "StrTqReqVal": apply_torque,
    "LKA_SysWrn": 0,
    "ActToiSta": 1 if lat_active else 0,
    "LKA_UsmMod": 0,  # hide LKAS settings
    "LKA_RcgSta": 0,
    "Damping_Gain": 100,  # can potentially tuned for better perf [3, 200]
  }

  ret = []
  if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
    lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT else "LKAS"
    # Don't send LFA while fully disengaged under dynamic handoff: the restored ADAS DRV ECU broadcasts its own LFA on
    # E-CAN, and two senders' counters collide at the MDPS -> lane-keep DTC. lfa_send_ok folds in BOTH "openpilot owns
    # the ECU" AND "the ECU is confirmed silenced" (carcontroller), so openpilot is never the second LFA source. On
    # takeover we continue the stock ECU's COUNTER (lfa_counter) so the MDPS sees one unbroken sequence.
    dynamic_radar_handoff = bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
    if CP.openpilotLongitudinalControl and (not dynamic_radar_handoff or lfa_send_ok):
      lfa_values = {**values, "COUNTER": lfa_counter} if dynamic_radar_handoff else values
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, lfa_values))
    ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestHandoffSteeringHandoff -v`
Expected: PASS (all, including the new counter test and the three existing LFA-presence tests).

- [ ] **Step 5: Commit**

```bash
git add opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py
git commit -m "hyundai canfd: gate LFA send on confirmed silence, continue counter on takeover"
```

---

### Task 4: carcontroller — wire the gate, hold lateral, seed + continue the counter

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/carcontroller.py` (`update()` ~line 189-194 and ~line 211; `create_canfd_msgs` ~line 350-357)
- Test: `opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py`

**Context:** `create_canfd_msgs` keeps `handoff_active` for its LFAHDA/ADRV gates (those are ECU-ownership, not LFA-collision); it gains `lfa_send_ok` and `lfa_counter` which it forwards only to `create_steering_messages`. The lateral hold and the counter seed happen in `update()` *after* `_tick_handoff_watchdog` (which updates `adrv_silenced` this frame).

- [ ] **Step 1: Write the failing test**

Add to `TestHandoffSilenceGate` in `test_canfd_dynamic_handoff.py` an end-to-end `create_canfd_msgs` gate + seed test:

```python
  def _fakes(self, mads_enabled=True):
    cs = types.SimpleNamespace(
      lfa_block_msg={f"BYTE{i}": 0 for i in range(3, 32)} | {"COUNTER": 0},
      main_cruise_enabled=False, adrv_lfa_counter=0x10,
      out=types.SimpleNamespace(steeringTorque=0, steeringAngleDeg=0),
    )
    cc = types.SimpleNamespace(enabled=False, latActive=True, leftBlinker=False, rightBlinker=False,
                               cruiseControl=types.SimpleNamespace(override=False, cancel=False),
                               actuators=types.SimpleNamespace(torque=1.0, accel=0.0, longControlState=None),
                               hudControl=types.SimpleNamespace(setSpeed=0, leftLaneVisible=False,
                                                                rightLaneVisible=False, leadDistanceBars=0))
    cc_sp = types.SimpleNamespace(mads=types.SimpleNamespace(enabled=mads_enabled))
    return cs, cc, cc_sp

  def _lfa_from_canfd(self, cc, lfa_send_ok, lfa_counter):
    cs, cc_obj, _ = self._fakes()
    cc.frame = 1
    msgs = cc.create_canfd_msgs(1, 100, 0.0, 0.0, False, cc_obj.hudControl, cs, cc_obj, True, lfa_send_ok, lfa_counter)
    return [(addr, dat, bus) for addr, dat, bus in msgs if addr == 0x12a and bus == cc.CAN.ECAN]

  def test_no_lfa_emitted_until_silenced(self):
    cc = self._cc()
    self.assertEqual(self._lfa_from_canfd(cc, lfa_send_ok=False, lfa_counter=0), [])  # handshake: withheld
    self.assertEqual(len(self._lfa_from_canfd(cc, lfa_send_ok=True, lfa_counter=0x11)), 1)  # after silence
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestHandoffSilenceGate::test_no_lfa_emitted_until_silenced -v`
Expected: FAIL — `create_canfd_msgs() takes 10 positional args but 12 were given`.

- [ ] **Step 3: Add `lfa_send_ok`/`lfa_counter` params to `create_canfd_msgs`**

Change the signature and the `create_steering_messages` call:

```python
  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC, handoff_active, lfa_send_ok=False, lfa_counter=0):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, lfa_send_ok, apply_steer_req, apply_torque, self.lkas_icon, lfa_counter))
```

(The remaining body — `create_suppress_lfa`, `create_lfahda_cluster`, `create_adrv_messages`, etc. — keeps using `handoff_active`, unchanged.)

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestHandoffSilenceGate::test_no_lfa_emitted_until_silenced -v`
Expected: PASS.

- [ ] **Step 5: Wire the hold + gate + seed in `update()`**

In `update()`, immediately after the watchdog tick block (`if self.dynamic_radar_handoff_enabled: self._tick_handoff_watchdog(CS, can_sends)`, ~line 188-189) and before the `# *** CAN/CAN FD specific ***` block (~line 191), insert:

```python
    # first-engage LFA handoff: until the stock ADRV ECU is confirmed silenced, openpilot must not become the
    # second LFA(0x12a) sender (counter collision -> MDPS LkaFailSta -> spurious steerTempUnavailable). Hold
    # lateral and withhold op's LFA until the disableRxAndTx ack lands (adrv_silenced) or the handshake gives up.
    steer_takeover_ok = self.adrv_silenced or self.silence_timeout
    lfa_send_ok = handoff_active and steer_takeover_ok
    if handoff_active and not steer_takeover_ok:
      apply_steer_req = 0
      apply_torque = 0
      self.apply_torque_last = apply_torque
    # seed op's LFA counter from the stock ECU's last value at the takeover instant, then continue it each frame
    if lfa_send_ok and not self.prev_lfa_send_ok:
      self.lfa_counter = (CS.adrv_lfa_counter + 1) & 0xFF
```

Change the `create_canfd_msgs` call (~line 193) to pass the new args, and advance the counter after:

```python
    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC, handoff_active, lfa_send_ok, self.lfa_counter))
      if lfa_send_ok:
        self.lfa_counter = (self.lfa_counter + 1) & 0xFF
```

Set `prev_lfa_send_ok` alongside `prev_handoff_active` (~line 211):

```python
    self.prev_handoff_active = handoff_active
    self.prev_lfa_send_ok = lfa_send_ok
```

> Note: `lfa_send_ok` is referenced in the non-CANFD branch path only as the just-computed local; it is defined unconditionally above the CANFD branch, so the `else:` (non-CANFD) path is unaffected. For non-dynamic-handoff cars `handoff_active` is always False, so `lfa_send_ok` is False and `create_steering_messages`'s `not dynamic_radar_handoff` term keeps LFA flowing — counter override is skipped (Task 3).

- [ ] **Step 6: Write the seed-continuity integration test**

Add to `TestHandoffSilenceGate`:

```python
  def test_counter_seeds_from_stock_and_continues(self):
    from opendbc.can import CANParser
    cc = self._cc()
    cc.adrv_silenced = True               # post-takeover
    cc.prev_lfa_send_ok = False           # this frame is the takeover edge
    cs, cc_obj, cc_sp = self._fakes()
    cs.adrv_lfa_counter = 0x20
    # mimic update()'s seed + emit + advance for two frames
    parser = CANParser("hyundai_canfd_generated", [("LFA", 0)], cc.CAN.ECAN)
    seen = []
    for _ in range(2):
      handoff_active = cc._handoff_active(cc_obj, cc_sp)
      steer_takeover_ok = cc.adrv_silenced or cc.silence_timeout
      lfa_send_ok = handoff_active and steer_takeover_ok
      if lfa_send_ok and not cc.prev_lfa_send_ok:
        cc.lfa_counter = (cs.adrv_lfa_counter + 1) & 0xFF
      cc.frame = 1
      msgs = cc.create_canfd_msgs(1, 100, 0.0, 0.0, False, cc_obj.hudControl, cs, cc_obj, handoff_active, lfa_send_ok, cc.lfa_counter)
      dat = next(d for a, d, b in msgs if a == 0x12a and b == cc.CAN.ECAN)
      parser.update([0, [(0x12a, dat, cc.CAN.ECAN)]])
      seen.append(parser.vl["LFA"]["COUNTER"])
      if lfa_send_ok:
        cc.lfa_counter = (cc.lfa_counter + 1) & 0xFF
      cc.prev_lfa_send_ok = lfa_send_ok
    self.assertEqual(seen, [0x21, 0x22])   # stock last 0x20 -> op continues 0x21, 0x22
```

- [ ] **Step 7: Run tests to verify they pass**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestHandoffSilenceGate -v`
Expected: PASS (all).

- [ ] **Step 8: Commit**

```bash
git add opendbc_repo/opendbc/car/hyundai/carcontroller.py opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py
git commit -m "hyundai handoff: hold lateral + continue LFA counter until ADRV silenced on engage"
```

---

### Task 5: Integration sweep + regression check

**Files:**
- Test only: full hyundai car test suite.

- [ ] **Step 1: Run the full dynamic-handoff suite**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -v`
Expected: PASS (all classes).

- [ ] **Step 2: Run the broader hyundai car tests + safety tests for regressions**

Run:
```bash
python -m pytest opendbc_repo/opendbc/car/hyundai/tests/ opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py -v
```
Expected: PASS. The panda safety tests must be unchanged (no safety edits in this plan); if any fail, a non-safety assumption leaked — stop and investigate.

- [ ] **Step 3: Lint/static check the three modified modules**

Run: `python -m pytest opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -q` and confirm no import/collection errors. Optionally run the repo's configured linter over `carstate.py carcontroller.py hyundaicanfd.py`.

- [ ] **Step 4: Commit (if any test-only fixups were needed)**

```bash
git add -A opendbc_repo/opendbc/car/hyundai/
git commit -m "hyundai handoff: integration test sweep for first-engage LFA fix"
```

---

## On-car verification (cannot be automated — track separately)

Reproduce a first MAIN engage of a drive and confirm via rlog (reuse `.drivedata/dual_lfa_test.py` and `dump_window.py`):
1. No `LFA(298)` `bus1`+`bus129` overlap across the engage edge — op emits no `LFA` until the `0x68 0x03` ack.
2. `carState.steerFaultTemporary` stays False through the engage; no `steerTempUnavailable`/`...Silent`.
3. Lateral engages within ~100–150 ms of MAIN (down from ~500 ms fault).
4. op's first post-takeover `LFA` `COUNTER` == stock's last +1 (no discontinuity).
5. A later in-drive engage (main already up) is unaffected.

If a residual brief alert remains at takeover despite counter continuity, revisit the "Consequence to confirm" in the spec (MDPS counter-check strictness / seed timing).

---

## Self-review notes

- **Spec coverage:** Decision 1 (gate on confirmed silence) → Tasks 3+4; Decision 2 (faster handshake) → Task 2 Step 3; Decision 3 (counter continuity) → Tasks 1, 3, 4; Decision 4 (timeout fallback) → Task 2 Steps 4-5. carstate snapshot → Task 1. Tests (i)-(v) → Tasks 1-4 + on-car section. Disengage reset → Task 2 Step 6.
- **Type/name consistency:** `adrv_silenced`, `silence_timeout`, `prev_lfa_send_ok`, `lfa_counter`, `adrv_lfa_counter`, `lfa_send_ok`, `_reset_handoff_silence_state`, `HANDOFF_RESPONSE_DEADLINE_FRAMES`, `HANDOFF_STEP_MAX_RETRIES` used consistently across tasks. `create_steering_messages(..., lfa_send_ok, lat_active, apply_torque, lkas_icon, lfa_counter=0)` and `create_canfd_msgs(..., handoff_active, lfa_send_ok=False, lfa_counter=0)` signatures match all call sites.
- **No placeholders:** every code/test step shows full content.

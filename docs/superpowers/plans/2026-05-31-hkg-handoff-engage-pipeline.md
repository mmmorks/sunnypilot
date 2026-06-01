# Component 2: Pipeline the Engage-Time UDS Silencing — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Shrink the stock-SCC↔openpilot `SCC_CONTROL` overlap at engage by not waiting for the extended-session ack before sending the silencing frame — the watchdog watches only the final (silencing) command.

**Architecture:** In the Hyundai carcontroller's dynamic-handoff watchdog, mark the engage sequence's first step (`0x10 03` extendedSession) **fire-and-forget**: it is sent, then dropped on the next tick without waiting for its `0x50` ack, so the watched `0x28 03` (disableRxAndTx) goes out one frame later instead of after the session ack round-trip. Only one request is ever outstanding (parser stays safe); a failed session-establish still surfaces as the `0x28 03` NRC. Disengage stays fully sequential.

**Tech Stack:** Python, opendbc (`opendbc_repo` submodule), unittest. Op's `SCC_CONTROL` is untouched (gated on `CC.enabled`, never gapped).

**Spec:** `docs/superpowers/specs/2026-05-31-hkg-handoff-main-engage-op-long-design.md` (Component 2 section).

**Repos:** Code change is in the `opendbc_repo` submodule. The main `sunnypilot` repo gets a submodule-pointer bump.

---

## Background the implementer needs

In `opendbc_repo/opendbc/car/hyundai/carcontroller.py`:
- `HandoffFault = structs.CarStateSP.HandoffFault` (module level, line 20). Values: `none`, `engageFailed`, `disengageFailed`.
- `_make_handoff_step(self, msg, expected, nrc_service)` (≈219-223) builds a step dict:
  `{'msg', 'expected', 'nrc_service', 'sent_frame': None, 'deadline': None, 'retries_left': self.HANDOFF_STEP_MAX_RETRIES}`.
- The engage edge (≈182-189) sets `self._handoff_seq = [<0x10 03 step>, <0x28 03 step>]` and `self._handoff_seq_kind = 1`.
  The disengage edge (≈168-175) sets `[<0x28 00 step>, <0x10 01 step>]`, kind `2`.
- `_tick_handoff_watchdog(self, CS, can_sends)` (≈225-273): each tick it consumes at most one new `0x738` response
  (`CS.adas_drv_uds_response_count` delta → `response_byte1`/`response_byte2`, `response_is_nrc = byte1 == 0x7F`),
  then for the head step: if unsent → append `step['msg']` to `can_sends`, set `sent_frame`/`deadline`; else match
  `response_byte1 == expected` (pop), NRC (`response_is_nrc and response_byte2 == nrc_service` → clear seq + latch),
  or `frame > deadline` (retry by resetting `sent_frame`/`deadline` while `retries_left`, else latch).
- UDS request byte layout (`opendbc/car/__init__.py`): session-control `dat = [0x02, 0x10, sf, ...]`; comm-control
  `dat = [0x03, 0x28, sf, comm_type, ...]`. So `dat[1]` is the service id. Messages are `CanData(addr, dat, bus)`,
  unpackable as `addr, dat, bus`.
- Constants: `HANDOFF_RESPONSE_DEADLINE_FRAMES = 50`, `HANDOFF_STEP_MAX_RETRIES = 3`.

Test build pattern (from `TestHandoffClusterHandoff._build_controller` in
`opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py`):
```python
from opendbc.car.hyundai.carcontroller import CarController
CP = _handoff_car_params(handoff=True)
CP_SP = CarInterface.get_non_essential_params_sp(CP, HANDOFF_CAR)
cc = CarController({"pt": "hyundai_canfd_generated", "cam": "hyundai_canfd_generated"}, CP, CP_SP)
```

How to run opendbc tests (from `opendbc_repo/`): `python -m pytest opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -v`.
Full safety/car suite is heavier; the `libsafety` "undefined symbol: set_safety_hooks" failure is a known flaky
parallel-build race — just re-run.

---

## File structure

- Modify: `opendbc_repo/opendbc/car/hyundai/carcontroller.py`
  - `_make_handoff_step` — add `fire_and_forget` field.
  - Extract `_engage_handoff_seq()` and `_disengage_handoff_seq()` methods (makes the edge wiring testable); call them from the engage/disengage edge blocks in `update`.
  - `_tick_handoff_watchdog` — drop already-sent fire-and-forget steps before the match logic.
- Modify: `opendbc_repo/opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py` — add a `TestHandoffEnginePipeline` class.
- Modify (Task 2): `sunnypilot` main repo — bump the `opendbc_repo` submodule pointer.

---

### Task 1: Fire-and-forget the engage session step (opendbc_repo)

Work in `/Users/john/Code/sunnypilot/opendbc_repo`. Create branch `hkg-handoff-engage-pipeline` first
(`git checkout -b hkg-handoff-engage-pipeline`); confirm the working tree is clean.

**Files:**
- Modify: `opendbc/car/hyundai/carcontroller.py`
- Modify: `opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py`

- [ ] **Step 1: Write the failing tests**

Append this class to `opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py` (before the
`if __name__ == "__main__":` block). Add `import types` is already imported at top; add
`from opendbc.car.hyundai.carcontroller import CarController, HandoffFault` to the imports if not present.

```python
class TestHandoffEnginePipeline(unittest.TestCase):
  """Engage silencing is pipelined: the extendedSession (0x10 03) preamble is fire-and-forget, so the watched
  disableRxAndTx (0x28 03) goes out one frame later instead of after the session ack round-trip. Only the
  silencing frame's response is watched; a failed session-establish surfaces as its NRC. Disengage stays
  fully sequential."""

  def _cc(self):
    CP = _handoff_car_params(handoff=True)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, HANDOFF_CAR)
    return CarController({"pt": "hyundai_canfd_generated", "cam": "hyundai_canfd_generated"}, CP, CP_SP)

  @staticmethod
  def _cs(count=0, byte1=0, byte2=0):
    return types.SimpleNamespace(adas_drv_uds_response_count=count,
                                 adas_drv_uds_response_byte1=byte1,
                                 adas_drv_uds_response_byte2=byte2)

  def _tick(self, cc, cs, frame):
    cc.frame = frame
    sends = []
    cc._tick_handoff_watchdog(cs, sends)
    return sends

  def test_engage_seq_first_step_is_fire_and_forget(self):
    cc = self._cc()
    seq = cc._engage_handoff_seq()
    self.assertTrue(seq[0]['fire_and_forget'])    # 0x10 03 extendedSession
    self.assertFalse(seq[1]['fire_and_forget'])   # 0x28 03 disableRxAndTx (watched)

  def test_disengage_seq_fully_watched(self):
    cc = self._cc()
    seq = cc._disengage_handoff_seq()
    self.assertFalse(seq[0]['fire_and_forget'])
    self.assertFalse(seq[1]['fire_and_forget'])

  def test_silencing_frame_sent_without_waiting_for_session_ack(self):
    cc = self._cc()
    cc._handoff_seq = cc._engage_handoff_seq()
    cc._handoff_seq_kind = 1
    cs = self._cs()  # no response ever delivered
    # frame 0: session control (0x10) goes out
    sends0 = self._tick(cc, cs, 0)
    self.assertEqual(len(sends0), 1)
    self.assertEqual(sends0[0][1][1], 0x10)
    # frame 1: WITHOUT any 0x50 ack, the preamble is dropped and the silencing frame (0x28) goes out
    sends1 = self._tick(cc, cs, 1)
    self.assertEqual(len(sends1), 1)
    self.assertEqual(sends1[0][1][1], 0x28)

  def test_success_on_silencing_ack(self):
    cc = self._cc()
    cc._handoff_seq = cc._engage_handoff_seq()
    cc._handoff_seq_kind = 1
    cs = self._cs()
    self._tick(cc, cs, 0)   # send 0x10 03
    self._tick(cc, cs, 1)   # drop preamble, send 0x28 03
    # deliver the 0x68 positive ack to disableRxAndTx
    ack = self._cs(count=1, byte1=0x68)
    self._tick(cc, ack, 2)
    self.assertEqual(cc._handoff_seq, [])
    self.assertEqual(cc.handoff_fault, HandoffFault.none)

  def test_engage_failed_on_silencing_nrc(self):
    cc = self._cc()
    cc._handoff_seq = cc._engage_handoff_seq()
    cc._handoff_seq_kind = 1
    cs = self._cs()
    self._tick(cc, cs, 0)
    self._tick(cc, cs, 1)
    nrc = self._cs(count=1, byte1=0x7F, byte2=0x28)  # NRC for service 0x28
    self._tick(cc, nrc, 2)
    self.assertEqual(cc.handoff_fault, HandoffFault.engageFailed)

  def test_silencing_timeout_retries_before_latch(self):
    cc = self._cc()
    cc._handoff_seq = cc._engage_handoff_seq()
    cc._handoff_seq_kind = 1
    cs = self._cs()
    self._tick(cc, cs, 0)            # 0x10 03
    self._tick(cc, cs, 1)            # 0x28 03 sent at frame 1, deadline = 1 + 50
    # past the deadline with no response -> the step is reset for retry (no send on this tick), not latched
    self._tick(cc, cs, 1 + cc.HANDOFF_RESPONSE_DEADLINE_FRAMES + 1)
    self.assertEqual(cc.handoff_fault, HandoffFault.none)
    self.assertTrue(cc._handoff_seq)  # still pending (retrying, not latched)
    # the following tick re-sends the silencing frame (sent_frame was reset to None)
    resend = self._tick(cc, cs, 1 + cc.HANDOFF_RESPONSE_DEADLINE_FRAMES + 2)
    self.assertEqual(len(resend), 1)
    self.assertEqual(resend[0][1][1], 0x28)
    self.assertEqual(cc.handoff_fault, HandoffFault.none)
```

- [ ] **Step 2: Run the tests, verify they fail**

From `opendbc_repo/`:
Run: `python -m pytest opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestHandoffEnginePipeline -v`
Expected: FAIL — `AttributeError: 'CarController' object has no attribute '_engage_handoff_seq'` (and `KeyError: 'fire_and_forget'`).

- [ ] **Step 3: Implement — add `fire_and_forget` to step dicts**

In `opendbc/car/hyundai/carcontroller.py`, change `_make_handoff_step`:

```python
  def _make_handoff_step(self, msg, expected, nrc_service, fire_and_forget=False):
    return {'msg': msg, 'expected': expected, 'nrc_service': nrc_service, 'fire_and_forget': fire_and_forget,
            'sent_frame': None, 'deadline': None, 'retries_left': self.HANDOFF_STEP_MAX_RETRIES}
```

- [ ] **Step 4: Implement — extract sequence builders and use them on the edges**

Add these two methods near `_make_handoff_step` (keep the existing explanatory comments from the edge blocks
attached to the builders):

```python
  def _engage_handoff_seq(self):
    # extendedSession is fire-and-forget: we don't wait for its 0x50 ack before sending the watched
    # disableRxAndTx, so the stock SCC is silenced ~one session round-trip sooner. A failed session
    # establish surfaces as the disableRxAndTx NRC.
    return [
      self._make_handoff_step(make_diagnostic_session_control_msg(0x730, self.CAN.ECAN, sub_function=0x03, suppress_response=False),
                              expected=0x50, nrc_service=0x10, fire_and_forget=True),
      self._make_handoff_step(make_communication_control_msg(0x730, self.CAN.ECAN, sub_function=0x03, suppress_response=False),
                              expected=0x68, nrc_service=0x28),
    ]

  def _disengage_handoff_seq(self):
    return [
      self._make_handoff_step(make_communication_control_msg(0x730, self.CAN.ECAN, sub_function=0x00, suppress_response=False),
                              expected=0x68, nrc_service=0x28),
      self._make_handoff_step(make_diagnostic_session_control_msg(0x730, self.CAN.ECAN, sub_function=0x01, suppress_response=False),
                              expected=0x50, nrc_service=0x10),
    ]
```

Then in `update`, replace the inline engage block (the `if engage_edge:` body that built `self._handoff_seq = [...]`)
with:

```python
    if engage_edge:
      self._handoff_seq = self._engage_handoff_seq()
      self._handoff_seq_kind = 1
```

and the inline disengage block (`if disengage_edge:`) with:

```python
    if disengage_edge:
      self._handoff_seq = self._disengage_handoff_seq()
      self._handoff_seq_kind = 2
```

(Preserve the existing explanatory comments above each `if` block.)

- [ ] **Step 5: Implement — drop sent fire-and-forget steps in the watchdog tick**

In `_tick_handoff_watchdog`, immediately BEFORE the `if self._handoff_seq:` block that sends/matches the head step
(but AFTER the response-consume block that sets `response_byte1`/`response_byte2`/`response_is_nrc`), insert:

```python
    # Drop already-sent fire-and-forget steps without waiting for their ack (the extendedSession preamble):
    # the watched silencing step then goes out the same tick. Any response collision is harmless because only
    # the watched step is matched.
    while self._handoff_seq and self._handoff_seq[0]['fire_and_forget'] and self._handoff_seq[0]['sent_frame'] is not None:
      self._handoff_seq.pop(0)
```

Leave the existing `if self._handoff_seq:` send/match block unchanged below it (fire-and-forget steps never reach
the match branch because they are popped here once sent).

- [ ] **Step 6: Run the new tests, verify they pass**

Run: `python -m pytest opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py::TestHandoffEnginePipeline -v`
Expected: PASS — all 6 tests. Confirm a real "6 passed", exit 0.

- [ ] **Step 7: Run the full handoff test file for regressions**

Run: `python -m pytest opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -v`
Expected: PASS — the pre-existing LFA/LFAHDA/canValid tests plus the new class. (If `libsafety` errors with
"undefined symbol: set_safety_hooks", that's the known parallel-build race — re-run.)

- [ ] **Step 8: Commit (in opendbc_repo)**

```bash
git add opendbc/car/hyundai/carcontroller.py opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py
git commit -m "hyundai canfd handoff: pipeline engage silencing (fire-and-forget session control)

The engage extendedSession (0x10 03) preamble no longer blocks the watched
disableRxAndTx (0x28 03): it is sent then dropped next tick without waiting
for its 0x50 ack, so the stock SCC is silenced ~one session round-trip
sooner and the stock/op SCC_CONTROL overlap at engage shrinks. Only one
request is outstanding at a time (0x738 parser stays safe); a failed session
establish still surfaces as the disableRxAndTx NRC. Disengage unchanged."
```

---

### Task 2: Bump the opendbc_repo submodule (main sunnypilot repo)

**Files:**
- Modify: `sunnypilot` repo submodule gitlink for `opendbc_repo`.

- [ ] **Step 1: Branch the main repo and stage the bump**

From `/Users/john/Code/sunnypilot` (confirm on `master`; if a parallel session left it elsewhere, `git checkout master` only if clean):

```bash
git checkout -b hkg-handoff-engage-pipeline
git add opendbc_repo
git status   # should show "modified: opendbc_repo (new commits)"
```

- [ ] **Step 2: Verify the gitlink points at the new opendbc commit**

```bash
git -C opendbc_repo rev-parse HEAD          # the Task 1 commit SHA
git diff --cached opendbc_repo              # Subproject commit ... -> <Task 1 SHA>
```
Expected: the staged new SHA equals the opendbc Task 1 commit.

- [ ] **Step 3: Commit the bump**

```bash
git commit -m "build: bump opendbc_repo for HKG CAN-FD handoff engage-silencing pipeline"
```

---

## On-car validation (after merge, not a code step)

Re-run the `SCC_CONTROL` dual-source analysis on a new drive (the `/tmp/fca_drives/scc_fight.py` method): confirm the
per-transition stock/op overlap shrank versus the 40-80 ms baseline, and that no `adasDrvHandoffEngageFail`
(`engageFailed`) events fired (which would indicate the `0x28 03` NRC'd because the session wasn't established in
time — if that happens, fall back to waiting one extra frame before the silencing frame, or revert to sequential).

## Self-review notes

- Spec coverage: pipelines the engage silencing per Component 2, scoped to engage only (disengage sequential),
  watching only the final command — exactly the agreed design. No two-in-flight parser rework (avoided via the
  fire-and-forget single-outstanding approach).
- Method/field names consistent across tasks: `_engage_handoff_seq`, `_disengage_handoff_seq`, `fire_and_forget`.
- Op `SCC_CONTROL` gating untouched — no zero-SCC gap.

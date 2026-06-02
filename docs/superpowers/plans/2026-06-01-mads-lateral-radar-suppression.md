# MADS-Lateral Radar Suppression Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On HKG CAN-FD dynamic-radar-handoff platforms, suppress the stock ADAS DRV ECU (radar/SCC/AEB) whenever openpilot has *any* control — MADS lateral or longitudinal — so the stock radar is active only when openpilot is fully disengaged.

**Architecture:** Replace the longitudinal-only "engaged" predicate that gates ECU ownership with an "owns-the-ECU-role" predicate = longitudinal OR latched-lateral authority, applied symmetrically on both layers: panda (`controls_allowed || controls_allowed_lateral`) and carcontroller (`CC.enabled || CC_SP.mads.enabled`). The strict longitudinal-accel gate stays on `controls_allowed` / `CC.enabled` so lateral-only never commands acceleration. During MADS-lateral, openpilot impersonates the silenced ECU exactly like the upstream disengaged-but-impersonating case (inactive SCC_CONTROL, ADRV, LFA, LFAHDA_CLUSTER).

**Tech Stack:** C safety code (`opendbc/safety`), Python carcontroller (`opendbc/car/hyundai`), pytest + a cffi-compiled libsafety test harness (auto-builds on import; safety suite enforces 100% line coverage).

**Spec:** `docs/superpowers/specs/2026-06-01-mads-lateral-radar-suppression-design.md`

**Working directory for all paths below:** `/Users/john/Code/sunnypilot/opendbc_repo`

**Branch:** create a feature branch off `master` before Task 1 (e.g. `git checkout -b mads-lateral-radar-suppression`).

---

## File Structure

- `opendbc/safety/modes/hyundai_canfd.h` — panda TX/forward hooks. Modify three gates: `handoff_blocked`, `disableRxAndTx` allow, `fwd_hook`.
- `opendbc/safety/tests/test_hyundai_canfd.py` — panda safety tests (class `TestHyundaiCanfdLKASteeringLongDynamicHandoff`). Add lateral-authority cases.
- `opendbc/car/hyundai/hyundaicanfd.py` — message builders. Rename `create_steering_messages` `enabled` param → `handoff_active` (its only use is the LFA-on-E-CAN gate).
- `opendbc/car/hyundai/carcontroller.py` — add `_handoff_active` helper, `prev_handoff_active`, retarget handoff edges + tester-present + impersonation gates.
- `opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py` — carcontroller tests. Add MADS-lateral cases; update direct `create_*` call sites for the new param.

---

## Task 1: panda — allow suppression + impersonation under `controls_allowed_lateral`

**Files:**
- Modify: `opendbc/safety/modes/hyundai_canfd.h` (lines ~187, ~239, ~287)
- Test: `opendbc/safety/tests/test_hyundai_canfd.py` (class `TestHyundaiCanfdLKASteeringLongDynamicHandoff`, ~line 316)

The test harness exposes `self.safety.set_controls_allowed_lateral(bool)` and `init_tests()` resets it to False. Existing tests leave it False, so their `controls_allowed=False → blocked` assertions stay valid after the change (`!(False || False)` is still `True`).

- [ ] **Step 1: Write the failing tests**

Add these methods to `TestHyundaiCanfdLKASteeringLongDynamicHandoff`:

```python
  def test_dynamic_handoff_scc_control_allowed_with_controls_allowed_lateral(self):
    """Inactive SCC_CONTROL must be accepted when only controls_allowed_lateral is set (MADS-lateral)."""
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    msg = self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, {"aReqRaw": 0.0, "aReqValue": 0.0})
    self.assertTrue(self._tx(msg))

  def test_dynamic_handoff_scc_control_nonzero_blocked_with_only_lateral(self):
    """A real (non-zero) accel SCC_CONTROL must still be rejected when only controls_allowed_lateral is set."""
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    msg = self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, {"aReqRaw": 1.0, "aReqValue": 1.0})
    self.assertFalse(self._tx(msg))

  def test_dynamic_handoff_adrv_0x160_allowed_with_controls_allowed_lateral(self):
    """ADRV impersonation must be accepted when only controls_allowed_lateral is set."""
    from opendbc.safety.tests.libsafety import libsafety_py as _lspy
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    msg = _lspy.make_CANPacket(0x160, 1, b'\x00' * 16)
    self.assertTrue(self._tx(msg))

  def test_uds_730_silencing_frame_allowed_with_controls_allowed_lateral(self):
    """disableRxAndTx must be accepted on a MADS-lateral engage (controls_allowed_lateral set)."""
    from opendbc.safety.tests.libsafety import libsafety_py as _lspy
    silencing_frame = b"\x03\x28\x03\x01\x00\x00\x00\x00"
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._tx(_lspy.make_CANPacket(0x730, 1, silencing_frame)))

  def test_fwd_hook_scc_control_blocked_when_controls_allowed_lateral(self):
    """When MADS-lateral owns the ECU role, the stock SCC_CONTROL must NOT be forwarded."""
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1A0))

  def test_fwd_hook_adrv_0x160_blocked_when_controls_allowed_lateral(self):
    """When MADS-lateral owns the ECU role, stock ADRV 0x160 must NOT be forwarded."""
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x160))
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/safety/tests/test_hyundai_canfd.py -k DynamicHandoff -v
```

Expected: the 6 new tests FAIL (`assertTrue` getting False on the allow cases; the fwd_hook cases returning a forward bus instead of -1). The non-zero-blocked test may already pass (accel check unchanged) — that's fine.

> Note: if the run dies with `undefined symbol: set_safety_hooks`, that's the known parallel-build race in libsafety — just re-run.

- [ ] **Step 3: Edit `handoff_blocked` (line ~187)**

```c
  bool handoff_blocked = false;
  if (hyundai_canfd_dynamic_handoff && !(controls_allowed || controls_allowed_lateral)) {
    if (((msg->addr == 0x1a0U) && hyundai_longitudinal) || hyundai_canfd_handoff_adrv_addr(msg->addr)) {
      handoff_blocked = true;
    }
  }
```

Update the comment above it to: `// Block openpilot's SCC_CONTROL/ADRV impersonation only when openpilot owns neither authority (fully disengaged). Under MADS-lateral (controls_allowed_lateral) the stock ECU is silenced, so openpilot must impersonate it; the accel check below still requires inactive accel without controls_allowed.`

- [ ] **Step 4: Edit the `disableRxAndTx` allow (line ~239)**

```c
                             ((GET_BYTES(msg, 0, 4) == 0x01032803U) && (controls_allowed || controls_allowed_lateral)));  // 0x28 disableRxAndTx (engage 2) — silences: lat or long authority
```

Update the silencing-frame comment (lines ~230-233) to note the gate is now "lateral OR longitudinal authority" — the silencing frame is permitted while openpilot owns the car under either authority, never when fully disengaged.

- [ ] **Step 5: Edit the `fwd_hook` block (line ~287)**

```c
      block = !hyundai_canfd_dynamic_handoff || controls_allowed || controls_allowed_lateral;
```

Update the comment to: `// With dynamic handoff: block stock forwarding when openpilot owns the ECU role under either authority (lat or long).`

- [ ] **Step 6: Run the new tests + the full dynamic-handoff class to verify**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/safety/tests/test_hyundai_canfd.py -v
```

Expected: PASS, including all pre-existing tests (their `controls_allowed_lateral` defaults to False, so disengaged-blocked assertions still hold).

- [ ] **Step 7: Run the full safety suite with the coverage gate**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo/opendbc/safety/tests
./test.sh
```

Expected: all safety tests pass AND `SUCCESS: All checked files have 100% coverage!`. The edits are inline modifications of existing lines (no new functions/branches as separate lines), so line coverage is preserved by the existing + new tests.

- [ ] **Step 8: Commit**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
git add opendbc/safety/modes/hyundai_canfd.h opendbc/safety/tests/test_hyundai_canfd.py
git commit -m "hyundai canfd handoff: allow radar suppression + ECU impersonation under controls_allowed_lateral

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: carcontroller — `handoff_active` predicate, edges, tester-present

**Files:**
- Modify: `opendbc/car/hyundai/carcontroller.py` (lines ~80, ~105-148, ~205)
- Test: `opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py`

`CC_SP.mads.enabled` is the latched MADS-engaged flag (stays True through blinker-pause); it is in scope inside `update()`. `create_canfd_msgs`/`create_steering_messages` do not receive `CC_SP`, so compute `handoff_active` once in `update()` and pass it down.

- [ ] **Step 1: Write the failing test for the predicate**

Add a new test class to `test_canfd_dynamic_handoff.py`:

```python
class TestHandoffActivePredicate(unittest.TestCase):
  """handoff_active = dynamic_radar_handoff_enabled AND (CC.enabled OR CC_SP.mads.enabled).
  This is the 'openpilot owns the ADAS DRV ECU' predicate that drives silence/restore + impersonation."""
  def _cc_ctrl(self, handoff=True):
    CP = _handoff_car_params(handoff)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, HANDOFF_CAR)
    return CarController({"pt": "hyundai_canfd_generated", "cam": "hyundai_canfd_generated"}, CP, CP_SP)

  @staticmethod
  def _cc_ccsp(enabled, mads_enabled):
    cc = types.SimpleNamespace(enabled=enabled)
    cc_sp = types.SimpleNamespace(mads=types.SimpleNamespace(enabled=mads_enabled))
    return cc, cc_sp

  def test_active_on_long_only(self):
    cc_ctrl = self._cc_ctrl()
    cc, cc_sp = self._cc_ccsp(enabled=True, mads_enabled=False)
    self.assertTrue(cc_ctrl._handoff_active(cc, cc_sp))

  def test_active_on_mads_lateral_only(self):
    cc_ctrl = self._cc_ctrl()
    cc, cc_sp = self._cc_ccsp(enabled=False, mads_enabled=True)
    self.assertTrue(cc_ctrl._handoff_active(cc, cc_sp))

  def test_inactive_when_fully_disengaged(self):
    cc_ctrl = self._cc_ctrl()
    cc, cc_sp = self._cc_ccsp(enabled=False, mads_enabled=False)
    self.assertFalse(cc_ctrl._handoff_active(cc, cc_sp))

  def test_inactive_without_handoff_flag(self):
    cc_ctrl = self._cc_ctrl(handoff=False)
    cc, cc_sp = self._cc_ccsp(enabled=True, mads_enabled=True)
    self.assertFalse(cc_ctrl._handoff_active(cc, cc_sp))
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -k HandoffActivePredicate -v
```

Expected: FAIL with `AttributeError: 'CarController' object has no attribute '_handoff_active'`.

- [ ] **Step 3: Add the `_handoff_active` helper**

In `opendbc/car/hyundai/carcontroller.py`, add a method on `CarController` (place it just above `update`):

```python
  def _handoff_active(self, CC, CC_SP):
    # openpilot owns the ADAS DRV ECU role under EITHER authority: MADS lateral (latched, blinker-pause-stable)
    # or longitudinal engage. Mirrors panda's controls_allowed || controls_allowed_lateral.
    return self.dynamic_radar_handoff_enabled and (CC.enabled or CC_SP.mads.enabled)
```

- [ ] **Step 4: Run the predicate test to verify it passes**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -k HandoffActivePredicate -v
```

Expected: PASS (4 tests).

- [ ] **Step 5: Wire the predicate into `update()` — edges, tester-present, prev-state**

In `update()`, initialize `handoff_active` right after the `MadsCarController.update(...)` call (line ~108), so it is available before the edge logic:

```python
    handoff_active = self._handoff_active(CC, CC_SP)
```

Replace the edge lines (~138-139):

```python
    disengage_edge = self.prev_handoff_active and not handoff_active
    engage_edge = not self.prev_handoff_active and handoff_active
```

Replace the tester-present gate term (~148): change `(not self.dynamic_radar_handoff_enabled or CC.enabled)` to `(not self.dynamic_radar_handoff_enabled or handoff_active)`.

Change the `prev_enabled` init (line ~80) from `self.prev_enabled = False` to `self.prev_handoff_active = False`, and the trailing assignment (line ~205) from `self.prev_enabled = CC.enabled` to `self.prev_handoff_active = handoff_active`. (`prev_enabled` has no other uses — confirm with `grep -n prev_enabled opendbc/car/hyundai/carcontroller.py` returns nothing after the edit.)

Pass `handoff_active` into the CAN-FD builder — change the call (~187) to:

```python
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC, handoff_active))
```

(The `create_canfd_msgs` signature change is implemented in Task 3; this call site is updated now and will fail import/collection until Task 3 — that is expected and resolved within this plan. To keep Task 2 independently runnable, temporarily give `create_canfd_msgs` a `handoff_active=False` default in its current signature, removed in Task 3 Step 3.)

- [ ] **Step 6: Write the edge regression test**

Add to `TestHandoffActivePredicate`:

```python
  def test_engage_edge_fires_on_mads_lateral(self):
    """A MADS-lateral engage (mads.enabled rising, CC.enabled False) must arm the engage silencing sequence."""
    cc_ctrl = self._cc_ctrl()
    self.assertFalse(cc_ctrl.prev_handoff_active)
    # simulate the edge bookkeeping update() performs
    cc, cc_sp = self._cc_ccsp(enabled=False, mads_enabled=True)
    active = cc_ctrl._handoff_active(cc, cc_sp)
    engage_edge = not cc_ctrl.prev_handoff_active and active
    self.assertTrue(engage_edge)
    cc_ctrl.prev_handoff_active = active
    # adding longitudinal later must NOT re-fire an engage edge
    cc2, cc_sp2 = self._cc_ccsp(enabled=True, mads_enabled=True)
    active2 = cc_ctrl._handoff_active(cc2, cc_sp2)
    self.assertFalse(not cc_ctrl.prev_handoff_active and active2)
```

- [ ] **Step 7: Run the predicate + edge tests**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -k HandoffActivePredicate -v
```

Expected: PASS (5 tests).

- [ ] **Step 8: Commit**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
git add opendbc/car/hyundai/carcontroller.py opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py
git commit -m "hyundai canfd handoff: drive silence/restore edges off handoff_active (lat or long)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: carcontroller — gate ECU impersonation messages on `handoff_active`

**Files:**
- Modify: `opendbc/car/hyundai/hyundaicanfd.py` (`create_steering_messages`, lines ~39, ~57)
- Modify: `opendbc/car/hyundai/carcontroller.py` (`create_canfd_msgs`, lines ~344, ~351, ~360-362, ~368-377)
- Test: `opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py`

This moves the four impersonated E-CAN broadcasts (LFA-on-E-CAN, LFAHDA_CLUSTER, ADRV, SCC_CONTROL) from "send while `CC.enabled`" to "send while `handoff_active`". `create_acc_control` keeps `CC.enabled` as its *active* flag (inactive accel during MADS-lateral).

- [ ] **Step 1: Update the LFA + LFAHDA existing tests for the lateral case (failing)**

In `test_canfd_dynamic_handoff.py`:

In `TestHandoffSteeringHandoff`, rename the helper arg for clarity and add the lateral case:

```python
  def _lfa_count(self, CP, handoff_active):
    packer = CANPacker("hyundai_canfd_generated")
    CAN = CanBus(CP)
    msgs = hyundaicanfd.create_steering_messages(packer, CP, CAN, handoff_active, handoff_active, 0, 0)
    return sum(1 for addr, _, bus in msgs if addr == LFA and bus == CAN.ECAN)

  def test_no_lfa_when_fully_disengaged_under_handoff(self):
    self.assertEqual(self._lfa_count(_handoff_car_params(handoff=True), handoff_active=False), 0)

  def test_lfa_present_when_handoff_active(self):
    # handoff_active is True for MADS-lateral OR longitudinal; LFA-on-E-CAN must be present either way.
    self.assertEqual(self._lfa_count(_handoff_car_params(handoff=True), handoff_active=True), 1)

  def test_lfa_always_present_without_handoff(self):
    CP = _handoff_car_params(handoff=False)
    self.assertEqual(self._lfa_count(CP, handoff_active=False), 1)
    self.assertEqual(self._lfa_count(CP, handoff_active=True), 1)
```

Delete the old `test_no_lfa_when_disengaged_under_handoff` / `test_lfa_present_when_engaged_under_handoff` (replaced above).

In `TestHandoffClusterHandoff`, update `_fakes` to also carry `CC_SP.mads` and update `_lfahda_count` to pass `handoff_active`, plus add a MADS-lateral case:

```python
  @staticmethod
  def _fakes(enabled, mads_enabled=False):
    cs = types.SimpleNamespace(lfa_block_msg={f"BYTE{i}": 0 for i in range(3, 32)} | {"COUNTER": 0},
                               main_cruise_enabled=False)
    cc = types.SimpleNamespace(enabled=enabled, leftBlinker=False, rightBlinker=False,
                               cruiseControl=types.SimpleNamespace(override=False))
    cc_sp = types.SimpleNamespace(mads=types.SimpleNamespace(enabled=mads_enabled))
    return cs, cc, cc_sp

  def _lfahda_count(self, enabled, handoff=True, mads_enabled=False):
    cc_ctrl = self._build_controller(handoff)
    cc_ctrl.frame = 5
    cs, cc, cc_sp = self._fakes(enabled, mads_enabled)
    handoff_active = cc_ctrl._handoff_active(cc, cc_sp)
    msgs = cc_ctrl.create_canfd_msgs(False, 0, 0, 0.0, False, types.SimpleNamespace(), cs, cc, handoff_active)
    return sum(1 for addr, _, _ in msgs if addr == self.LFAHDA)

  def test_no_lfahda_cluster_when_fully_disengaged_under_handoff(self):
    self.assertEqual(self._lfahda_count(enabled=False, handoff=True, mads_enabled=False), 0)

  def test_lfahda_cluster_present_when_engaged_under_handoff(self):
    self.assertEqual(self._lfahda_count(enabled=True, handoff=True), 1)

  def test_lfahda_cluster_present_on_mads_lateral_under_handoff(self):
    self.assertEqual(self._lfahda_count(enabled=False, handoff=True, mads_enabled=True), 1)
```

Also add an SCC_CONTROL/ADRV impersonation test to `TestHandoffClusterHandoff` (or a new class) confirming they appear during MADS-lateral:

```python
  def test_scc_and_adrv_present_on_mads_lateral(self):
    cc_ctrl = self._build_controller(handoff=True)
    cc_ctrl.frame = 0  # %2==0 so acc_control path runs
    cs, cc, cc_sp = self._fakes(enabled=False, mads_enabled=True)
    handoff_active = cc_ctrl._handoff_active(cc, cc_sp)
    msgs = cc_ctrl.create_canfd_msgs(False, 0, 0, 0.0, False, types.SimpleNamespace(), cs, cc, handoff_active)
    addrs = {addr for addr, _, _ in msgs}
    self.assertIn(0x1a0, addrs)   # SCC_CONTROL impersonated (inactive)
    self.assertIn(0x160, addrs)   # an ADRV broadcast
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -k "Steering or Cluster" -v
```

Expected: FAIL — `create_steering_messages` still gates LFA on the old `enabled` semantics for the lateral arg only via `enabled`, and `create_canfd_msgs` either rejects the extra positional arg or still gates on `CC.enabled`, so the MADS-lateral cases return 0 / missing addrs.

- [ ] **Step 3: Rename `create_steering_messages` param and retarget the LFA-on-E-CAN gate**

In `opendbc/car/hyundai/hyundaicanfd.py`, change the signature (line ~39):

```python
def create_steering_messages(packer, CP, CAN, handoff_active, lat_active, apply_torque, lkas_icon):
```

and the LFA-on-E-CAN gate (line ~57):

```python
    if CP.openpilotLongitudinalControl and (not dynamic_radar_handoff or handoff_active):
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
```

Update the comment at lines ~54-55 to: `# Don't send LFA while fully disengaged under dynamic handoff: the restored ADAS DRV ECU broadcasts its own LFA on E-CAN and the two counters collide at the MDPS. While openpilot owns the ECU (handoff_active: MADS-lateral or longitudinal) the ECU is silenced, so openpilot is the sole LFA source.`

- [ ] **Step 4: Retarget `create_canfd_msgs` to `handoff_active`**

In `opendbc/car/hyundai/carcontroller.py`, change the signature (line ~344) — remove the temporary default added in Task 2:

```python
  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC, handoff_active):
```

Pass `handoff_active` as the steering arg (line ~351):

```python
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, handoff_active, apply_steer_req, apply_torque, self.lkas_icon))
```

LFAHDA_CLUSTER gate (line ~361): change `(not self.dynamic_radar_handoff_enabled or CC.enabled)` to `(not self.dynamic_radar_handoff_enabled or handoff_active)`.

ADRV + acc_control gate (line ~369): change `if not (self.dynamic_radar_handoff_enabled and not CC.enabled):` to `if not (self.dynamic_radar_handoff_enabled and not handoff_active):`. Leave the inner `create_acc_control(..., CC.enabled, ...)` call passing `CC.enabled` unchanged — that is the active flag (inactive accel under MADS-lateral).

- [ ] **Step 5: Run the impersonation tests to verify they pass**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py -v
```

Expected: PASS (all classes, including the new MADS-lateral LFA / LFAHDA / SCC+ADRV cases and the unchanged pipeline/UDS tests).

- [ ] **Step 6: Commit**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
git add opendbc/car/hyundai/hyundaicanfd.py opendbc/car/hyundai/carcontroller.py opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py
git commit -m "hyundai canfd handoff: impersonate ECU (LFA/LFAHDA/ADRV/SCC) during MADS-lateral

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: full verification

**Files:** none (verification only).

- [ ] **Step 1: Run the hyundai car test module**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/ -v
```

Expected: PASS. Pay attention to `test_hyundai.py` (fingerprint/param tests) for any unexpected interaction.

- [ ] **Step 2: Run the full safety suite + coverage gate**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo/opendbc/safety/tests
./test.sh
```

Expected: all pass; `SUCCESS: All checked files have 100% coverage!`.

- [ ] **Step 3: Lint the changed Python**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
ruff check opendbc/car/hyundai/carcontroller.py opendbc/car/hyundai/hyundaicanfd.py opendbc/car/hyundai/tests/test_canfd_dynamic_handoff.py
```

Expected: no errors. (Watch for now-unused `enabled` imports/locals or `noqa` drift in `create_acc_control` — unchanged here, but verify.)

- [ ] **Step 4: Document the engage-fault behavior change (no code)**

Confirm in review that `adasDrvHandoffEngageFail` (immediate disable, `selfdrived.py:255-256`) is now reachable on a MADS-lateral engage if the silencing UDS fails. This is intended conservative behavior: on a silence failure the stock AEB stays alive, and openpilot disengages (lateral included). No code change; record it in the PR description.

- [ ] **Step 5: Final commit / PR prep**

The implementation lives in the `opendbc_repo` submodule. Per the project's submodule-push rules, push the opendbc branch to its origin before bumping the superrepo gitlink. Prepare the PR body summarizing: the owns-the-ECU-role predicate, the four impersonated broadcasts, the retained no-accel-without-`controls_allowed` invariant, and the accepted loss of stock AEB during MADS-lateral.

---

## Self-Review notes

- **Spec coverage:** panda gates 1–3 → Task 1; accel-limit unchanged (criterion 5) → asserted in Task 1 Step 1 non-zero test; carcontroller predicate/edges/tester-present → Task 2; LFA/LFAHDA/ADRV/SCC impersonation (criterion 4) → Task 3; full-disengage restore (criterion 2) → covered by the `not handoff_active` paths tested in Tasks 2/3; engage-fault consequence → Task 4 Step 4.
- **Latched-signal decision:** enforced by gating on `CC_SP.mads.enabled` / `controls_allowed_lateral`, never instantaneous `CC.latActive`.
- **Type consistency:** `_handoff_active(CC, CC_SP)` and the `handoff_active` positional arg into `create_canfd_msgs` / `create_steering_messages` are used identically across Tasks 2–3; the Task 2 temporary default is explicitly removed in Task 3 Step 4.

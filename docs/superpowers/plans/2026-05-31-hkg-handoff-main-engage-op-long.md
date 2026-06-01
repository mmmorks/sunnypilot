# `main` Engages Openpilot Longitudinal (Component 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On HKG CAN-FD dynamic-handoff (and any op-long) cars, pressing the cruise **main** button engages openpilot longitudinal + lateral at the current speed, instead of engaging only MADS lateral while the stock SCC does the speed.

**Architecture:** Add a small, pure, testable helper `main_button_engages_op(...)` in `selfdrived.py` (mirroring the existing `mute_can_loss_at_shutdown` helper) that adds `EventName.buttonEnable` to the event set on the `cruiseState.available` rising edge when the car is op-long and the user has MADS Unified Engagement Mode + `MadsMainCruiseAllowed` enabled. Call it from `SelfdriveD.update_events`, **before** the op state machine consumes events (line ~618). Op engaging at the current speed (`initialize_v_cruise` seeds from `vEgo`) triggers the existing handoff engage-silencing of the stock SCC, so no separate stock suppression is needed.

**Tech Stack:** Python, openpilot/sunnypilot `selfdrived`, cereal `log.OnroadEvent.EventName`, pytest.

**Spec:** `docs/superpowers/specs/2026-05-31-hkg-handoff-main-engage-op-long-design.md`

**Scope:** Component 1 only. Component 2 (pipeline the engage silencing) is a separate plan.

---

## Background the implementer needs

- `SelfdriveD.step()` (`selfdrive/selfdrived/selfdrived.py:614-625`) runs in this order each cycle:
  `update_events(CS)` (line 616, clears `self.events` at the top, line 199) →
  `state_machine.update(self.events)` (618, **op consumes events here** → sets `self.enabled`) →
  `mads.update(CS)` (620) → `CS_prev = CS` (625).
  Therefore the engage event MUST be added during `update_events`, before 618. The MADS module
  runs at 620 (too late for op).
- `EventName.buttonEnable` maps to `ET.ENABLE`; the op state machine transitions `disabled →
  enabled` on `ET.ENABLE` unless an `ET.NO_ENTRY` event is present. Gas raises only
  `gasPressedOverride` (no `NO_ENTRY`), so engaging with the gas down works; brake raises
  `pedalPressed` / `preEnableStandstill` (`NO_ENTRY`) and still blocks — matching stock.
- On engage, `initialize_v_cruise` (`selfdrive/car/cruise.py:141-152`) seeds the set-speed from
  `vEgo` when no accel/resume button is in play (a `main` press has none) — i.e. current speed.
- The existing MADS lateral-on-main trigger uses exactly this rising edge
  (`sunnypilot/mads/mads.py:166-168`): `CS.cruiseState.available and not
  self.selfdrive.CS_prev.cruiseState.available`. We reuse the identical signal.
- Params already read into MADS (`sunnypilot/mads/mads.py:55-58`):
  `self.mads.main_enabled_toggle` = `MadsMainCruiseAllowed`,
  `self.mads.unified_engagement_mode` = `MadsUnifiedEngagementMode`. `self.mads` is reachable from
  `SelfdriveD` (`self.mads`, set at `selfdrived.py:189`). `self.CS_prev` is initialized in
  `__init__` (used at `selfdrived.py:245`) and updated at end of `step()`.
- `EventName` and `Events` are already imported in `selfdrived.py` (used by the shutdown mute).

**Test environment note (from project memory):** importing `selfdrived.py` pulls
`msgq.visionipc.visionipc_pyx` (compiled). If pytest fails to import it, build once:
`uv pip install "imgui @ git+https://github.com/commaai/dependencies.git@release-imgui#subdirectory=imgui"`
then `python -m SCons -u -j8 msgq_repo/msgq/visionipc/visionipc_pyx.so` (venv SCons). The
existing `selfdrive/selfdrived/tests/test_shutdown_can_mute.py` imports from `selfdrived.py` the
same way, so if that suite runs, this one will too.

---

## File structure

- Modify: `selfdrive/selfdrived/selfdrived.py` — add `main_button_engages_op` helper (near
  `mute_can_loss_at_shutdown`, ~line 70) and call it inside `update_events` (after the
  `resumeBlocked` block, ~line 237).
- Create: `selfdrive/selfdrived/tests/test_main_button_engages_op.py` — unit tests for the helper.

---

### Task 1: Helper `main_button_engages_op` + unit tests

**Files:**
- Create: `selfdrive/selfdrived/tests/test_main_button_engages_op.py`
- Modify: `selfdrive/selfdrived/selfdrived.py` (add helper ~line 70)

- [ ] **Step 1: Write the failing test**

Create `selfdrive/selfdrived/tests/test_main_button_engages_op.py`:

```python
from cereal import log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.selfdrived import main_button_engages_op

EventName = log.OnroadEvent.EventName

# All gates satisfied, on the cruise-available rising edge (the main-button press).
ENGAGE = dict(op_long=True, unified_engagement=True, main_allowed=True,
              cruise_available=True, cruise_available_prev=False)


def make_events():
  return Events()


class TestMainButtonEngagesOp:
  def test_engages_on_main_rising_edge(self):
    events = make_events()
    assert main_button_engages_op(events, **ENGAGE)
    assert EventName.buttonEnable in events.names

  def test_no_engage_when_not_op_long(self):
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "op_long": False})
    assert EventName.buttonEnable not in events.names

  def test_no_engage_without_unified_engagement(self):
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "unified_engagement": False})
    assert EventName.buttonEnable not in events.names

  def test_no_engage_without_main_allowed(self):
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "main_allowed": False})
    assert EventName.buttonEnable not in events.names

  def test_no_engage_without_rising_edge(self):
    # cruise already available last cycle -> not a fresh main press
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "cruise_available_prev": True})
    assert EventName.buttonEnable not in events.names

  def test_no_engage_when_cruise_unavailable(self):
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "cruise_available": False})
    assert EventName.buttonEnable not in events.names
```

- [ ] **Step 2: Run the test, verify it fails**

Run: `source .venv/bin/activate && python -m pytest selfdrive/selfdrived/tests/test_main_button_engages_op.py -v`
Expected: FAIL — `ImportError: cannot import name 'main_button_engages_op'`.
(If it instead fails on `visionipc_pyx`, build it per the test-environment note above, then re-run.)

- [ ] **Step 3: Implement the helper**

In `selfdrive/selfdrived/selfdrived.py`, immediately after the `mute_can_loss_at_shutdown`
function (the block ending ~line 70), add:

```python
def main_button_engages_op(events, *, op_long, unified_engagement, main_allowed,
                           cruise_available, cruise_available_prev):
  engage = (op_long and unified_engagement and main_allowed
            and cruise_available and not cruise_available_prev)
  if engage:
    events.add(EventName.buttonEnable)
  return engage
```

- [ ] **Step 4: Run the test, verify it passes**

Run: `source .venv/bin/activate && python -m pytest selfdrive/selfdrived/tests/test_main_button_engages_op.py -v`
Expected: PASS — all 6 tests green.

- [ ] **Step 5: Commit**

```bash
git add selfdrive/selfdrived/selfdrived.py selfdrive/selfdrived/tests/test_main_button_engages_op.py
git commit -m "selfdrived: add main_button_engages_op helper

Pure helper that emits buttonEnable on the cruise-available rising edge
for op-long cars with MADS UEM + MadsMainCruiseAllowed. Tested in isolation."
```

---

### Task 2: Wire the helper into `SelfdriveD.update_events`

**Files:**
- Modify: `selfdrive/selfdrived/selfdrived.py` (inside `update_events`, after the `resumeBlocked` block ~237)

- [ ] **Step 1: Add the call site**

In `selfdrive/selfdrived/selfdrived.py`, find the `resumeBlocked` block in `update_events`:

```python
    # Block resume if cruise never previously enabled
    resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
    if not self.CP.pcmCruise and CS.vCruise > 250 and resume_pressed:
      self.events.add(EventName.resumeBlocked)
```

Immediately after it, add:

```python
    # main-button engages openpilot longitudinal (unified) for op-long cars with MADS UEM + main allowed
    main_button_engages_op(self.events,
                           op_long=self.CP.openpilotLongitudinalControl,
                           unified_engagement=self.mads.unified_engagement_mode,
                           main_allowed=self.mads.main_enabled_toggle,
                           cruise_available=CS.cruiseState.available,
                           cruise_available_prev=self.CS_prev.cruiseState.available)
```

- [ ] **Step 2: Lint**

Run: `source .venv/bin/activate && ruff check selfdrive/selfdrived/selfdrived.py`
Expected: no errors (or only pre-existing, unrelated ones).

- [ ] **Step 3: Run the selfdrived test suite for regressions**

Run: `source .venv/bin/activate && python -m pytest selfdrive/selfdrived/tests/ -v`
Expected: PASS — existing tests (incl. `test_shutdown_can_mute.py`) and the new
`test_main_button_engages_op.py` all green. No regressions.

- [ ] **Step 4: Commit**

```bash
git add selfdrive/selfdrived/selfdrived.py
git commit -m "selfdrived: engage openpilot on main-button for op-long MADS UEM cars

Call main_button_engages_op in update_events before the op state machine
so a main-cruise press engages op longitudinal (+lateral, unified) at the
current speed, instead of leaving the stock SCC to do longitudinal. Gated
on openpilotLongitudinalControl + MadsUnifiedEngagementMode +
MadsMainCruiseAllowed. Op's engagement silences the stock SCC via the
existing handoff engage sequence."
```

---

## On-car validation (after merge, not a code step)

The unit tests cover the gating logic; the engage→silence behavior is validated on-car:
1. Start a drive, press **main** (no prior SET this drive). Confirm op engages longitudinal +
   lateral immediately (cluster/UI shows openpilot longitudinal, not stock cruise).
2. Confirm engaging with the gas pressed works (op enters overriding, holds the press-moment
   speed after gas release).
3. Pull the rlog and re-run `/tmp/fca_drives/longcheck.py`-style analysis: during the main-engaged
   window, op is the sole `SCC_CONTROL` (src≥128) source and stock (src 1) is silenced; the
   stock/op dual-source overlap stays in the ~40-80 ms/transition range (no sustained fight).

---

## Self-review notes

- Spec coverage: helper + call site implement Component 1's seam and gating; tests cover each
  gate and the rising-edge requirement. Component 2 explicitly out of scope (separate plan).
- The call site reuses the exact rising-edge signal MADS already uses for main lateral engage,
  so behavior is consistent with the existing MADS path.
- Param mapping confirmed (`main_enabled_toggle`/`unified_engagement_mode`). `self.CS_prev` and
  `self.mads` confirmed available in `SelfdriveD.update_events`.

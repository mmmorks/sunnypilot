# HKG main-button engages openpilot longitudinal (derived panda flag) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a cruise-`main` press engage openpilot longitudinal on op-long Hyundai cars: a derived safety flag (`op_long ∧ Mads ∧ UEM ∧ MainCruiseAllowed`) drives both the panda's `controls_allowed` grant on the `acc_main_on` toggle and `selfdrived`'s `main_button_engages_op`, so the two never desync and no `controlsMismatch` fires.

**Architecture:** One derived flag, three coordinated edits. (1) New `HyundaiSafetyFlagsSP.MAIN_ENGAGES_OP_LONG` derived in `helpers.py`. (2) The panda reads it into a global and grants/revokes `controls_allowed` at the existing `acc_main_on` toggle in `hyundai_common_cruise_buttons_check`. (3) `selfdrived`'s `main_button_engages_op` gains a `mads_enabled` gate so its condition equals the flag's. Because the flag == selfdrived's condition, panda-grant ⟺ op-engage always, so the dynamic-handoff forward hook never silences the stock SCC/AEB without op controlling.

**Tech Stack:** Python (sunnypilot `helpers.py`, `selfdrived.py`; Hyundai SP `values.py`), C (panda safety `hyundai_common.h`), unittest safety + selfdrived tests, git submodule.

**Spec:** `docs/superpowers/specs/2026-06-01-hkg-handoff-main-grants-longitudinal-panda-design.md`

**Coverage gate:** `opendbc/safety/tests/test.sh` enforces `--fail-under-line=100` (excluding `libsafety`). The new grant line must run both ways (grant + revoke); the flag-clear path is covered by existing tests.

**Memory note (flake):** a parallel libsafety build race can surface `undefined symbol: set_safety_hooks`; if seen, just re-run.

**Submodule note:** the Hyundai SP values + panda C live in `opendbc_repo`; `helpers.py` + `selfdrived.py` live in the `sunnypilot` main repo. Commit `opendbc_repo` first, then bump the gitlink (Task 6).

---

## File structure

- Modify: `opendbc_repo/opendbc/sunnypilot/car/hyundai/values.py` — add `MAIN_ENGAGES_OP_LONG = 16` to `HyundaiSafetyFlagsSP`.
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_common.h` — C enum bit, global, init read, grant at the `acc_main_on` toggle.
- Modify: `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py` — panda grant/revoke + regression tests.
- Modify: `sunnypilot/mads/helpers.py` — derive the flag.
- Modify: `selfdrive/selfdrived/selfdrived.py` — add `mads_enabled` gate to `main_button_engages_op` + caller.
- Modify: `selfdrive/selfdrived/tests/test_main_button_engages_op.py` — `mads_enabled` cases.

---

## Task 1: Define the derived flag (Python + C enum)

**Files:**
- Modify: `opendbc_repo/opendbc/sunnypilot/car/hyundai/values.py:11-15`
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_common.h:20-25`

- [ ] **Step 1: Add the Python SP flag bit**

In `opendbc_repo/opendbc/sunnypilot/car/hyundai/values.py`, in `class HyundaiSafetyFlagsSP`, after
`NON_SCC = 8` add:

```python
  MAIN_ENGAGES_OP_LONG = 16
```

- [ ] **Step 2: Add the matching C enum bit**

In `opendbc_repo/opendbc/safety/modes/hyundai_common.h`, in the `enum { HYUNDAI_PARAM_SP_* }`
(lines 20-25), after `HYUNDAI_PARAM_SP_NON_SCC = 8,` add:

```c
  HYUNDAI_PARAM_SP_MAIN_ENGAGES_OP_LONG = 16,
```

- [ ] **Step 3: Commit**

```bash
cd opendbc_repo
git add opendbc/sunnypilot/car/hyundai/values.py opendbc/safety/modes/hyundai_common.h
git commit -m "hyundai: add MAIN_ENGAGES_OP_LONG SP safety flag bit

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Panda grants/revokes controls_allowed on main when the flag is set

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_common.h`
- Test: `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py`

- [ ] **Step 1: Add the import to the test file**

In `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py`, after
`from opendbc.car.hyundai.values import HyundaiSafetyFlags` (line 5), add:

```python
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
```

- [ ] **Step 2: Write the failing positive test**

Add to `class TestHyundaiCanfdLFASteeringLong` (the concrete long, non-handoff class at ~line 289):

```python
  def test_main_cruise_engages_longitudinal(self):
    """With MAIN_ENGAGES_OP_LONG, a main-cruise press grants longitudinal controls_allowed (engage),
    and a second press (main off) revokes it — so a single main press engages op long instead of
    tripping controlsMismatch."""
    default_sp = self.safety.get_current_safety_param_sp()
    self.safety.set_current_safety_param_sp(default_sp | HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE
                                            | HyundaiSafetyFlagsSP.MAIN_ENGAGES_OP_LONG)
    self.safety.set_safety_hooks(self.safety.get_current_safety_mode(), self.safety.get_current_safety_param())

    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed())

    # main press (rising edge) -> acc_main_on on -> longitudinal authority granted
    self._rx(self._main_cruise_button_msg(0))
    self._rx(self._main_cruise_button_msg(1))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed())

    # second main press (toggle off) -> acc_main_on off -> longitudinal authority revoked
    self._rx(self._main_cruise_button_msg(0))
    self._rx(self._main_cruise_button_msg(1))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed())

    self.safety.set_current_safety_param_sp(default_sp)
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `cd opendbc_repo && python -m pytest opendbc/safety/tests/test_hyundai_canfd.py -k test_main_cruise_engages_longitudinal -v`
Expected: FAIL at `assertTrue(get_controls_allowed())` — main toggles `acc_main_on` but does not grant.
(If `undefined symbol: set_safety_hooks`, re-run — build-race flake.)

- [ ] **Step 4: Add the global + init read**

In `opendbc_repo/opendbc/safety/modes/hyundai_common.h`, next to the other SP globals (near
`bool hyundai_longitudinal_main_cruise_toggleable = false;`, ~line 57), add:

```c
extern bool hyundai_main_engages_op_long;
bool hyundai_main_engages_op_long = false;
```

In `hyundai_common_init`, next to the line that reads `hyundai_longitudinal_main_cruise_toggleable`
(line 90), add:

```c
  hyundai_main_engages_op_long = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_MAIN_ENGAGES_OP_LONG);
```

- [ ] **Step 5: Add the grant/revoke at the acc_main_on toggle**

In `hyundai_common_cruise_buttons_check`, change the toggle block (lines 146-148) from:

```c
    // toggle main cruise state on rising edge of main cruise button
    if (main_button && !main_button_prev && hyundai_longitudinal_main_cruise_toggleable) {
      acc_main_on = !acc_main_on;
    }
```

to:

```c
    // toggle main cruise state on rising edge of main cruise button
    if (main_button && !main_button_prev && hyundai_longitudinal_main_cruise_toggleable) {
      acc_main_on = !acc_main_on;

      // main-engages-op cars: main also grants/revokes longitudinal authority so a single main press
      // engages openpilot longitudinal (panda authority rises with selfdrived's enable). The flag is
      // derived to match selfdrived's main_button_engages_op exactly, so this never grants without op
      // engaging (which would silence the stock SCC under dynamic handoff). Symmetric: main-off revokes.
      if (hyundai_main_engages_op_long) {
        controls_allowed = acc_main_on;
      }
    }
```

- [ ] **Step 6: Run the positive test to verify it passes**

Run: `cd opendbc_repo && python -m pytest opendbc/safety/tests/test_hyundai_canfd.py -k test_main_cruise_engages_longitudinal -v`
Expected: PASS.

- [ ] **Step 7: Write the regression test (no flag → no grant)**

Add to the same `class TestHyundaiCanfdLFASteeringLong`:

```python
  def test_main_cruise_no_longitudinal_grant_without_flag(self):
    """Without MAIN_ENGAGES_OP_LONG, a main-cruise press toggles acc_main_on but must NOT grant
    longitudinal controls_allowed (regression guard)."""
    default_sp = self.safety.get_current_safety_param_sp()
    self.safety.set_current_safety_param_sp(default_sp | HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE)
    self.safety.set_safety_hooks(self.safety.get_current_safety_mode(), self.safety.get_current_safety_param())

    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._main_cruise_button_msg(0))
    self._rx(self._main_cruise_button_msg(1))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed())

    self.safety.set_current_safety_param_sp(default_sp)
```

- [ ] **Step 8: Run the regression test**

Run: `cd opendbc_repo && python -m pytest opendbc/safety/tests/test_hyundai_canfd.py -k test_main_cruise_no_longitudinal_grant_without_flag -v`
Expected: PASS.

- [ ] **Step 9: Commit**

```bash
cd opendbc_repo
git add opendbc/safety/modes/hyundai_common.h opendbc/safety/tests/test_hyundai_canfd.py
git commit -m "safety/hyundai: main grants longitudinal controls_allowed when MAIN_ENGAGES_OP_LONG

At the acc_main_on toggle, grant/revoke controls_allowed when the derived
MAIN_ENGAGES_OP_LONG flag is set, so a single main press engages op long
instead of tripping controlsMismatch. Flag matches selfdrived's engage
condition, so it never grants without op engaging.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Derive the flag in the car params (`helpers.py`)

**Files:**
- Modify: `sunnypilot/mads/helpers.py:53-62`

- [ ] **Step 1: Add the derivation in `set_car_specific_params`**

In `sunnypilot/mads/helpers.py`, inside the `if CP.brand == "hyundai":` block in
`set_car_specific_params`, after the existing `LONG_MAIN_CRUISE_TOGGLEABLE` lines (`:60`), add:

```python
    # Derived (not a user toggle): main-button engages openpilot longitudinal. Set only when op is
    # the longitudinal authority AND all of the user's MADS main-engage opts are on, so the panda's
    # controls_allowed grant on the acc_main_on edge exactly matches selfdrived's main_button_engages_op
    # (no desync -> never silences the stock SCC/AEB without op controlling).
    main_engages_op_long = (CP.openpilotLongitudinalControl
                            and params.get_bool("Mads")
                            and params.get_bool("MadsUnifiedEngagementMode")
                            and params.get_bool("MadsMainCruiseAllowed"))
    if main_engages_op_long:
      CP_SP.safetyParam |= HyundaiSafetyFlagsSP.MAIN_ENGAGES_OP_LONG
```

- [ ] **Step 2: Confirm `HyundaiSafetyFlagsSP` is imported in `helpers.py`**

Run: `cd ~/Code/sunnypilot && grep -n "HyundaiSafetyFlagsSP" sunnypilot/mads/helpers.py`
Expected: an existing import line (it is already used for `LONG_MAIN_CRUISE_TOGGLEABLE` at `:60`).
If absent, add `from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP` with the
other imports.

- [ ] **Step 3: Commit**

```bash
cd ~/Code/sunnypilot
git add sunnypilot/mads/helpers.py
git commit -m "mads/helpers: derive MAIN_ENGAGES_OP_LONG safety flag for op-long MADS main-engage

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Match selfdrived's engage condition to the flag (`mads_enabled` gate)

**Files:**
- Modify: `selfdrive/selfdrived/selfdrived.py:73-80,248-252`
- Test: `selfdrive/selfdrived/tests/test_main_button_engages_op.py`

- [ ] **Step 1: Update the failing test first**

Open `selfdrive/selfdrived/tests/test_main_button_engages_op.py`. The `ENGAGE` baseline dict
(near the top) currently has `op_long=True, unified_engagement=True, main_allowed=True,
cruise_available=True, cruise_available_prev=False`. Add `mads_enabled=True` to it:

```python
ENGAGE = dict(op_long=True, mads_enabled=True, unified_engagement=True, main_allowed=True,
              cruise_available=True, cruise_available_prev=False)
```

Then add a new test asserting `mads_enabled=False` blocks engagement (place it next to the other
gate tests):

```python
def test_mads_disabled_blocks_engage():
  events = Events()
  assert not main_button_engages_op(events, **{**ENGAGE, "mads_enabled": False})
  assert EventName.buttonEnable not in events.names
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd ~/Code/sunnypilot && .venv/bin/python -m pytest selfdrive/selfdrived/tests/test_main_button_engages_op.py -v`
Expected: FAIL — `main_button_engages_op` does not yet accept `mads_enabled` (TypeError: unexpected
keyword argument). (If import of `visionipc_pyx` fails, build it once per the spec's test note.)

- [ ] **Step 3: Add the `mads_enabled` parameter and gate**

In `selfdrive/selfdrived/selfdrived.py`, change `main_button_engages_op` (lines 73-79) from:

```python
def main_button_engages_op(events, *, op_long, unified_engagement, main_allowed,
                           cruise_available, cruise_available_prev):
  engage = (op_long and unified_engagement and main_allowed
            and cruise_available and not cruise_available_prev)
```

to:

```python
def main_button_engages_op(events, *, op_long, mads_enabled, unified_engagement, main_allowed,
                           cruise_available, cruise_available_prev):
  engage = (op_long and mads_enabled and unified_engagement and main_allowed
            and cruise_available and not cruise_available_prev)
```

- [ ] **Step 4: Update the caller**

In `selfdrive/selfdrived/selfdrived.py`, change the call (lines 248-252) to pass `mads_enabled`:

```python
    main_button_engages_op(self.events,
                           op_long=self.CP.openpilotLongitudinalControl,
                           mads_enabled=self.mads.enabled_toggle,
                           unified_engagement=self.mads.unified_engagement_mode,
                           main_allowed=self.mads.main_enabled_toggle,
                           cruise_available=CS.cruiseState.available,
                           cruise_available_prev=self.CS_prev.cruiseState.available)
```

- [ ] **Step 5: Run the selfdrived tests to verify they pass**

Run: `cd ~/Code/sunnypilot && .venv/bin/python -m pytest selfdrive/selfdrived/tests/test_main_button_engages_op.py -v`
Expected: PASS (all, including `test_mads_disabled_blocks_engage`).

- [ ] **Step 6: Commit**

```bash
cd ~/Code/sunnypilot
git add selfdrive/selfdrived/selfdrived.py selfdrive/selfdrived/tests/test_main_button_engages_op.py
git commit -m "selfdrived: gate main_button_engages_op on mads_enabled to match MAIN_ENGAGES_OP_LONG

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Full safety suite + 100% coverage

**Files:** none (verification only)

- [ ] **Step 1: Run the Hyundai safety tests**

Run: `cd opendbc_repo && python -m pytest opendbc/safety/tests/test_hyundai_canfd.py opendbc/safety/tests/test_hyundai.py -v`
Expected: PASS. The existing `test_main_cruise_button` (asserts only `controls_allowed_lateral`)
still passes. If `undefined symbol: set_safety_hooks`, re-run.

- [ ] **Step 2: Run the full safety suite with the coverage gate**

Run: `cd opendbc_repo/opendbc/safety/tests && ./test.sh`
Expected: all pass AND `SUCCESS: All checked files have 100% coverage!`. If coverage fails on the
new grant line, confirm the Task 2 positive test exercises both toggle-on (grant) and toggle-off
(revoke).

- [ ] **Step 3: No commit** (verification only). If anything fails, return to the failing task.

---

## Task 6: Bump the `sunnypilot` submodule + sanity checks

**Files:** Modify the `sunnypilot` gitlink for `opendbc_repo`.

- [ ] **Step 1: Confirm the target handoff fingerprint will set the flag**

The flag needs `op_long ∧ Mads ∧ UEM ∧ MainCruiseAllowed`, and the `acc_main_on` toggle needs
`LONG_MAIN_CRUISE_TOGGLEABLE` (set unconditionally for Hyundai in `helpers.py:56-60`). Sanity-check
the derivation reads the right params:

Run: `cd ~/Code/sunnypilot && grep -n "MainCruiseAllowed\|UnifiedEngagementMode\|get_bool(\"Mads\")\|openpilotLongitudinalControl" sunnypilot/mads/helpers.py`
Expected: the Task 3 derivation references all four. (UEM/MainCruiseAllowed/Mads are user params;
op_long is the car's longitudinal mode.)

- [ ] **Step 2: Verify opendbc_repo commits and bump the gitlink**

```bash
cd ~/Code/sunnypilot
git -C opendbc_repo log --oneline -3
git add opendbc_repo
git status   # expect: "modified: opendbc_repo (new commits)"
git commit -m "build: bump opendbc_repo for HKG main-engages-op-long panda flag

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

- [ ] **Step 3: Confirm no unintended submodule/worktree churn**

Run: `cd ~/Code/sunnypilot && git -C opendbc_repo status --short && git submodule status opendbc_repo`
Expected: clean opendbc_repo worktree; submodule status shows the new commit with no leading `+`/`-`.
(Per project memory, a main-repo `git checkout` can reset the submodule worktree — fix later with
`git submodule update` if it happens.)

- [ ] **Step 4 (optional, on-car): behavior verification**

On the next drive on the handoff car (Mads/UEM/MainCruiseAllowed all on): a single main press
engages op longitudinal, the stock SCC is silenced, and `controlsMismatch` no longer fires ~2 s
after a main-only engage (compare to today's `0000037f` / `00000380`). Also confirm: with MADS off,
a main press does nothing (SET still engages).

---

## Self-review notes

- **Spec coverage:** derived flag (Task 1 + Task 3); panda grant/revoke at `acc_main_on` toggle
  (Task 2); selfdrived condition match via `mads_enabled` (Task 4); behavior matrix rows are
  enforced by the flag==helper-condition equality (Tasks 2-4) + tests; tests incl. regression and
  100% coverage (Tasks 2,4,5); submodule bump (Task 6). All spec sections map to a task.
- **No placeholders:** every code/test step shows full code and an exact run command + expected result.
- **Type/name consistency:** `MAIN_ENGAGES_OP_LONG = 16` (Python `HyundaiSafetyFlagsSP`, Task 1
  Step 1) and `HYUNDAI_PARAM_SP_MAIN_ENGAGES_OP_LONG = 16` (C, Task 1 Step 2) share the value; the
  C global `hyundai_main_engages_op_long` is defined (Task 2 Step 4) and used (Step 5); the
  `mads_enabled` keyword matches across helper def (Task 4 Step 3), caller (Step 4), and tests
  (Step 1); `self.mads.enabled_toggle` is the `Mads` param (`mads.py:55`), matching the flag's
  `params.get_bool("Mads")` (Task 3).

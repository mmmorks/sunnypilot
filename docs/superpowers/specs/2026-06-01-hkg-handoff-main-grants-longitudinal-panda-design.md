# HKG main-button engages openpilot longitudinal — derived panda authority flag

Date: 2026-06-01 (revised)
Status: Design approved, pending implementation plan
Repos touched: `sunnypilot` (main: `helpers.py`, `selfdrived.py`) + `opendbc_repo` submodule (panda safety + Hyundai SP values)

## Problem

The shipped `main_button_engages_op` change (commits `ebb47f326c` + `f87bc63b20`, master HEAD
`77ee3cb52`) makes `selfdrived` flip openpilot to `enabled` on the cruise-`main` rising edge for
op-long MADS-UEM cars. Today's live HKG CAN-FD drives (2026-06-01, routes
`0000037f--f18904f139` and `00000380--7b696fe946`, device `comma-3a30a619`)
**force-disengage with a critical `controlsMismatch` ~2 s after every main-only engage**, and the
pre-change baseline `0000037e` (`c96cecc8d`) had zero `controlsMismatch`.

### Root cause

For op-long Hyundai cars the panda grants longitudinal `controls_allowed` only on the SET/RES
button falling edge (`hyundai_common.h:133-138`); the main button only toggles `acc_main_on`
(`:146-148`), never `controls_allowed`. openpilot cannot self-grant it (tx hook gates op's SET/RES
on `controls_allowed`, `hyundai_canfd.h:214`). So `selfdrived` flipping `enabled` in software with
no panda authority is guaranteed to trip `controlsMismatch` (`selfdrived.py:375-376,548-554`).
**There is no software-only fix — main-engages-longitudinal needs a panda change.** Verified
against upstream openpilot (`~/Code/openpilot` @ `1df90d143`): identical SET/RES-only grant, no
main path, identical `mismatch_counter→controlsMismatch` logic.

### The hard constraint that shapes this design

The panda grant and `selfdrived`'s engage **must fire on exactly the same condition**. If the
panda grants `controls_allowed` when `selfdrived` does NOT engage op, the dynamic-handoff forward
hook (`hyundai_canfd.h:287`: `block = !dynamic_handoff || controls_allowed`) **silences the stock
SCC/AEB (`0x1a0`) with nothing replacing it** — a safety regression. This rules out gating the
grant on anything coarser than `selfdrived`'s condition.

`selfdrived`'s engage condition uses `MadsUnifiedEngagementMode` (UEM) and `MadsMainCruiseAllowed`,
which **never reach the panda** (no safety param, no altExp bit — verified). The panda's own
`controls_allowed_lateral` only encodes "MADS on" (granted on any `acc_main` rising edge,
`mads.h:87-90,128`), so it cannot stand in for those toggles. Therefore the condition must be
**propagated to the panda as a single derived flag**.

## Goals

1. A cruise-`main` press engages openpilot longitudinal (+ lateral) on op-long Hyundai cars when
   the user's MADS main-engage opts are on, with the panda granting `controls_allowed` to match so
   no `controlsMismatch` fires. On dynamic-handoff cars this also silences the stock SCC via the
   existing engage pipeline.
2. The panda grant and `selfdrived` engage are **always in lockstep** (one derived flag drives
   both conditions), so `controls_allowed` is never True without op engaging → stock SCC/AEB is
   never silenced without op control.
3. The existing `MadsMainCruiseAllowed` / UEM / `Mads` toggles keep working: turning any off
   cleanly returns to the prior behavior (two-step, or main-inert).

## Non-goals

- **No new user-facing setting.** The flag is *derived* from existing params, not exposed.
- No change to SET/RES/CANCEL grant logic, to MADS lateral authority, or to Honda / non-Hyundai.
- Not addressing the separate `adasDrvHandoffEngageFail` "SP immediate-disable doesn't disengage"
  observation (tracked separately).

## Design

### 1. Derived safety flag

Add `HyundaiSafetyFlagsSP.MAIN_ENGAGES_OP_LONG = 16` (next free bit; `ESCC=1`,
`LONG_MAIN_CRUISE_TOGGLEABLE=2`, `HAS_LDA_BUTTON=4`, `NON_SCC=8`) and the matching C enum
`HYUNDAI_PARAM_SP_MAIN_ENGAGES_OP_LONG = 16`.

Derive it in `sunnypilot/mads/helpers.py:set_car_specific_params` (inside the existing
`CP.brand == "hyundai"` block), setting `CP_SP.safetyParam |= MAIN_ENGAGES_OP_LONG` iff:

```
CP.openpilotLongitudinalControl
  and params.get_bool("Mads")
  and params.get_bool("MadsUnifiedEngagementMode")
  and params.get_bool("MadsMainCruiseAllowed")
```

This is exactly `selfdrived`'s engage condition, so the panda and `selfdrived` are guaranteed to
agree. (Dynamic handoff is not part of the condition — the feature is valid for any op-long
Hyundai; on handoff cars the existing engage-silencing pipeline composes on top. Handoff is the
on-car-validated case.)

### 2. Panda grant

In `hyundai_common.h`: add `extern bool hyundai_main_engages_op_long` (default false), set it in
`hyundai_common_init` from `current_safety_param_sp`. At the `acc_main_on` toggle in
`hyundai_common_cruise_buttons_check` (`:146-148`), grant/revoke when the flag is set:

```c
if (main_button && !main_button_prev && hyundai_longitudinal_main_cruise_toggleable) {
  acc_main_on = !acc_main_on;
  if (hyundai_main_engages_op_long) {
    controls_allowed = acc_main_on;   // grant on toggle-on, revoke on toggle-off
  }
}
```

No function-signature change (uses the global, like `hyundai_longitudinal_main_cruise_toggleable`).
SET/RES/CANCEL grants are untouched. This lives in `hyundai_common`, so it applies to CAN and
CAN-FD op-long Hyundai equally; non-Hyundai (incl. Honda) is unaffected.

### 3. selfdrived condition match

Add a `mads_enabled` keyword to `main_button_engages_op` (`selfdrived.py:73`) and gate on it, so
the helper's condition equals the flag's (it currently omits the `Mads` check):

```python
def main_button_engages_op(events, *, op_long, mads_enabled, unified_engagement, main_allowed,
                           cruise_available, cruise_available_prev):
  engage = (op_long and mads_enabled and unified_engagement and main_allowed
            and cruise_available and not cruise_available_prev)
```

The caller (`:248`) passes `mads_enabled=self.mads.enabled_toggle` (the `Mads` param).

### Behavior matrix (why every toggle combo is safe)

| Mads | UEM | MainCruiseAllowed | flag | selfdrived on main | panda on main | net |
|---|---|---|---|---|---|---|
| on | on | on | **set** | engages op-long | grants `controls_allowed` | op does lat+long, stock silenced ✓ |
| on | off | on | clear | MADS lateral only (`mads.py:166`) | no grant → stock fwd | op lateral + stock long (two-step) ✓ |
| on | on/off | off | clear | nothing on main | no grant | main inert; SET engages ✓ |
| off | * | * | clear | nothing (helper gated on `mads_enabled`) | no grant | main inert; SET engages ✓ |

The flag (panda) and the helper (selfdrived) are clear/set together in every row → no desync → the
stock SCC/AEB is silenced only when op is actually the longitudinal authority.

## Testing

- **Panda** (`opendbc/safety/tests/test_hyundai_canfd.py`): with `MAIN_ENGAGES_OP_LONG +
  LONG_MAIN_CRUISE_TOGGLEABLE`, main rising edge grants `controls_allowed`, second press (toggle
  off) revokes; without `MAIN_ENGAGES_OP_LONG`, main does **not** grant (regression guard).
  100%-line-coverage gate (`test.sh --fail-under-line=100`) — exercise grant and revoke.
- **selfdrived** (`tests/test_main_button_engages_op.py`): `mads_enabled=False` blocks engage even
  when all else is set; existing engage cases updated to pass `mads_enabled=True`.

## Risks

- Granting longitudinal authority on a non-SET button: narrowly gated by a flag that equals
  `selfdrived`'s engage condition, so it only happens when op actually engages. Fork-only.
- `acc_main_on` (panda, button-toggled) vs `cruiseState.available` (selfdrived, from car) are
  distinct edges; the existing `acc_main_on` drift sync (`hyundai_common.h:191-193`) bounds
  divergence. Verify on-car.

## Evidence / references

- Drives `0000037f`, `00000380` (post-change), baseline `0000037e` (`c96cecc8d`).
- Panda: `hyundai_common.h:90,109-160,191-193`; `hyundai_canfd.h:214,287`; `mads.h:87-90,128`.
- selfdrived: `selfdrived.py:73-80,248-252,375-376,548-554`; `sunnypilot/mads/mads.py:55-62,166`.
- Flags: `opendbc/sunnypilot/car/hyundai/values.py` (`HyundaiSafetyFlagsSP`); `helpers.py:53-62`.
- Upstream cross-check: `~/Code/openpilot` `safety/safety/safety_hyundai_common.h:91-108`.

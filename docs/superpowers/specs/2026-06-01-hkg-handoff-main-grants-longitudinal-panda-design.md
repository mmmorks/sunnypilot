# HKG CAN-FD Dynamic Handoff — panda grants longitudinal authority on `main`

Date: 2026-06-01
Status: Design approved, pending implementation plan
Repos touched: `opendbc_repo` submodule (panda safety) — no `sunnypilot` main-repo behavior change

## Problem

The shipped `main_button_engages_op` change (commits `ebb47f326c` + `f87bc63b20`, master HEAD
`77ee3cb52`) makes `selfdrived` flip openpilot to `enabled` on the cruise-`main` rising edge for
op-long MADS-UEM cars. Today's live HKG CAN-FD drives (2026-06-01, routes
`0000037f--f18904f139` and `00000380--7b696fe946`, device `comma-3a30a619`) show it
**force-disengages with a critical `controlsMismatch` ~2 s after every main-only engage**:

- Drive `037f` t=25.3 s: `mainCruise` press → op `enabled`, `pandaState.controlsAllowed=False` →
  `mismatch_counter` climbs → `controlsMismatch` (immediateDisable) at t=27.3 s.
- Drive `0380` t=171.4 s: `mainCruise` at standstill (no pedals) → same, disabled at t=173.5 s.
- The engage only "stuck" when the driver additionally pressed SET/`decelCruise` (037f t=64.5 s →
  `controlsAllowed` flipped True at t=65.1 s).

Baseline pre-change drive `0000037e` (`c96cecc8d`) had **zero** `buttonEnable` and **zero**
`controlsMismatch`, confirming the disengagement is caused by this change.

### Root cause

For `hyundai_longitudinal` (op-long) cars the panda grants longitudinal `controls_allowed`
**only on the SET/RES button falling edge** read from the driver's stalk
(`opendbc/safety/modes/hyundai_canfd.h:105-118` → `hyundai_common.h:133-138`). The main button
only toggles `acc_main_on` (`hyundai_common.h:146-148`), never `controls_allowed`. And openpilot
**cannot self-grant** it: the tx hook (`hyundai_canfd.h:214`) blocks op from transmitting SET/RES
unless `controls_allowed` is already True. So `selfdrived` flipping `enabled` in software with no
panda authority is structurally guaranteed to trip `controlsMismatch`. There is **no
software-only fix** — main-engages-longitudinal requires a panda safety-model change.

The existing handoff engage-silencing also depends on `controls_allowed` (the panda only accepts
the `disableRxAndTx` silencing frame while `controls_allowed`, `hyundai_canfd.h:231-239`), so
granting authority on main is also what lets the stock SCC actually get silenced on a main press.

## Goals

1. On `CANFD_DYNAMIC_HANDOFF` cars, a cruise-`main` press grants the panda longitudinal
   `controls_allowed`, so `selfdrived`'s `enabled` and the panda's authority come up together —
   op engages fully (lateral + longitudinal) and the existing handoff silences the stock SCC.
2. Symmetric teardown: turning the system off via `main` revokes longitudinal authority.
3. No regression to SET/RES/CANCEL engagement, and no behavior change on any non-handoff car
   (notably Honda, where `main` is the off switch — `honda.h:151-152`).

## Non-goals

- No revert of the `selfdrived` `main_button_engages_op` change — it is **kept**; this spec adds
  the panda half that makes it work.
- No new safety flag / param propagation — reuse the existing `CANFD_DYNAMIC_HANDOFF` flag
  (decision below). No change to the MADS UEM / `MadsMainCruiseAllowed` gating in `selfdrived`.
- No change to SET/RES/CANCEL grant logic, to lateral (MADS) authority, or to Honda / non-CANFD
  Hyundai.
- Not addressing the separate `adasDrvHandoffEngageFail` "SP immediate-disable doesn't actually
  disengage" observation (tracked separately).

## Design

### Trigger (engage)
On the `acc_main_on` **rising** edge (driver's main-button press toggling the system on), when
the active config has `CANFD_DYNAMIC_HANDOFF` set: `controls_allowed = true`. This mirrors the
SET/RES falling-edge grant.

### Trigger (disengage)
On the `acc_main_on` **falling** edge (main pressed again → system off): `controls_allowed =
false`. CANCEL continues to revoke as today. Symmetric with the grant — "main turns the whole
system on/off."

### Location
The main button already toggles `acc_main_on` in `hyundai_common_cruise_buttons_check`
(`hyundai_common.h:146-148`). The grant/revoke is added at that toggle, guarded by the
dynamic-handoff flag. Because the flag (`hyundai_canfd_dynamic_handoff`) is CANFD-scoped while
`hyundai_common_cruise_buttons_check` is shared with CAN Hyundai, the implementation will either
(a) thread a `bool main_engages_long` parameter into `hyundai_common_cruise_buttons_check`, or
(b) perform the `controls_allowed` grant/revoke in the CANFD rx hook immediately around the
common call, keying off the `acc_main_on` edge. Choice made at plan time to keep `hyundai_common`
generic; **(a) is preferred** (single place, edge state already lives in common).

### Gating decision — reuse `CANFD_DYNAMIC_HANDOFF`
No new flag. The grant fires for every dynamic-handoff car. For a handoff user who has
`MadsMainCruiseAllowed` **off** (wants the two-step), the panda grants `controls_allowed` on main
but `selfdrived` does not emit `buttonEnable`, so op stays disengaged — a benign "armed but idle"
state: no actuation and no stock silencing (the handoff engage-silencing keys off `CC.enabled`,
not `controls_allowed`), until they press SET. The two-step still works; the early grant is inert.

### Coordination with selfdrived
`selfdrived`'s `buttonEnable` (cruise-available rising edge) and the panda grant (`acc_main_on`
rising edge) fire on the same physical main press, so `enabled` and `controls_allowed` rise
together and `mismatch_counter` never accumulates. The `acc_main_on` ↔ `acc_main_on_tx` sync /
mismatch path (`hyundai_common.h:191-193`) must remain consistent so the new grant does not trip
the acc-main mismatch counter — verified as part of implementation.

### Standstill & entry guards
The panda only *permits*; op's own state machine still decides when to actually command
(`preEnableStandstill` NO_ENTRY → "release brake to engage"; gear must be D; brake/reverse block).
So granting authority at standstill needs no special panda handling — it matches a SET press at
standstill today.

## Testing

Safety-code change → `opendbc/safety/tests/test_hyundai_canfd.py` gets:

- With `CANFD_DYNAMIC_HANDOFF`: `acc_main_on` rising edge grants `controls_allowed`; falling edge
  revokes; CANCEL revokes; SET/RES still grant.
- Without the flag (regression guard): main rising edge does **not** grant `controls_allowed`;
  SET/RES/CANCEL unchanged.
- acc-main-mismatch path unaffected by the new grant.

Run the full safety suite (`opendbc/safety` tests). Note the project memory flake: a parallel
libsafety build race can surface `undefined symbol: set_safety_hooks` — re-run if seen.

## Risks

- **Granting longitudinal authority on a non-SET button** is the sensitive part. Justified by:
  narrow gating to the fork-only `CANFD_DYNAMIC_HANDOFF` config; it mirrors what these cars' stock
  SCC already does on a main press (main activates ACC); and op's own entry guards remain in force.
  Fork-only — upstream comma would not accept main-engages-longitudinal.
- **Desync with the selfdrived MADS gate** for `MadsMainCruiseAllowed`-off users is analyzed above
  and is inert (no actuation without `enabled`).

## Evidence / references

- Live drives: `0000037f--f18904f139`, `00000380--7b696fe946` (post-change, HEAD `77ee3cb52`);
  baseline `0000037e--1c7939d8bd` (`c96cecc8d`). Device `comma-3a30a619`.
- Panda: `opendbc/safety/modes/hyundai_canfd.h:105-118,208-239`, `hyundai_common.h:109-160,191-193`,
  `honda.h:97-160` (contrast).
- selfdrived: `selfdrive/selfdrived/selfdrived.py` (`main_button_engages_op`, `mismatch_counter`
  375-376/548-554, `cruiseMismatch` 450-452).
- Prior specs: `2026-05-27-hkg-canfd-dynamic-radar-handoff-design.md`,
  `2026-05-31-hkg-handoff-main-engage-op-long-design.md`.

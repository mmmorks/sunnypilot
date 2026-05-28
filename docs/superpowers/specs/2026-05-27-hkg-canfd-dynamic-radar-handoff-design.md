# HKG CAN-FD HDA II Dynamic Radar Handoff — Design Spec

**Date:** 2026-05-27
**Status:** Draft, pending user review
**Target scope:** HKG CAN-FD platforms with HDA II (ADAS DRV ECU). Gated at runtime on `CP.flags & HyundaiFlags.CANFD_LKA_STEERING`, with `HyundaiFlags.CANFD_NO_RADAR_DISABLE` and `HyundaiFlags.CANFD_CAMERA_SCC` excluded. Platforms in scope today: Hyundai Ioniq 5 (HDA II), Ioniq 6, Kia EV6 (HDA II), K8 Hybrid HDA II, Niro EV 2025 HDA II, Sorento Hybrid 2026 HDA II, Genesis GV70 Electrified, Genesis G80 2nd Gen FL.
**Author conversation context:** sunnypilot fork of openpilot

---

## Problem

On HKG CAN-FD HDA II platforms, enabling sunnypilot's alpha longitudinal control puts the stock ADAS DRV ECU (`0x730`) into a permanent disabled state at boot via a UDS sequence:

1. Enter extended diagnostic session — `UDS 0x10 0x03`.
2. Communication control to disable TX/RX — `UDS 0x28 0x03 (disableRxAndTx)`.
3. Keep-alive via tester-present-with-suppress-response at 1 Hz — `UDS 0x3E 0x80` to `0x730` (`opendbc_repo/opendbc/car/hyundai/carcontroller.py:107-114`).

The disable persists for the entire onroad session, regardless of whether sunnypilot is engaged. Stock SCC and stock AEB are offline from the moment pandad and carcontroller start, until ignition off. There is no current mechanism to bring the ADAS DRV ECU back online when sunnypilot is not the longitudinal authority.

## Goal

Bring the stock ADAS DRV ECU online whenever openpilot is not the longitudinal authority. Take it offline (today's behavior) whenever openpilot is. The transition is driven by openpilot's engaged state, communicated to the panda safety code via the existing 100 Hz heartbeat that maintains the `controls_allowed` global.

The deinit is a single UDS frame: `UDS 0x28 0x00 (enableRxAndTx)` sent to `0x730`. This is the natural reverse of the boot-time disable. No ECU reset, no session timeout reliance.

## Non-goals

- **Preserving AEB while sunnypilot is actively engaged.** That is a different problem with a different solution (ESCC handles it on CAN; no CAN-FD equivalent exists today). Out of scope here.
- **Guaranteeing that stock AEB actually re-arms after the deinit.** The UDS sequence is mechanically clean; whether the ADAS DRV ECU's firmware reliably resumes arming AEB across enable/disable cycles is empirical. Characterized in Phase 1, validated in Phase 5.
- **Radar health monitoring or fault detection.** Best-effort mechanics only.
- **Generalization to non-HDA-II HKG CAN-FD platforms.** Those use a different disable target (`0x7D0` radar directly) and a potentially different UDS sequence. Framework is extensible per-platform but only HDA II is in scope here.
- **Generalization to legacy CAN HKG platforms.** ESCC is the existing solution for those.
- **A "panic recovery" fallback via UDS ECU Reset (`0x11`).** If the `0x28 0x00` deinit fails in practice, ECU reset is a possible follow-up mechanism but is not part of v1.

## User-facing promise

**Strong promise on mechanics:** every transition of openpilot's engaged state cleanly flips the ADAS DRV ECU between "disabled by openpilot" (engaged) and "online" (disengaged). On disengage, a single UDS CommunicationControl frame re-enables the ECU.

**No promise on hardware response:** the spec does not claim AEB will be restored across a transition. The UI sub-toggle's tooltip states this explicitly: *"AEB may not re-arm after disengagement until next ignition or OBD clear. Stock AEB behavior is hardware-dependent."*

## Architecture

Two cooperating mechanisms, both driven by openpilot's engaged state:

1. **Panda safety code branches on `controls_allowed`** (the existing global maintained by the 100 Hz heartbeat at `pandad.cc:306` → `panda.cc:138` → `opendbc/safety/safety.h:43, 527-530`). When `controls_allowed` is true, openpilot's longitudinal CAN frames (SCC_CONTROL and the ADRV addresses) are TX-allowed and stock SCC/FCA frames are blocked from forwarding. When `controls_allowed` is false, the inverse: longitudinal TX is rejected and stock SCC/FCA frames forward unmodified. A new `safety_param` bit, `HYUNDAI_PARAM_CANFD_DYNAMIC_HANDOFF` (1024 / 2¹⁰), opts the safety code into this branching; without the bit, today's static behavior is preserved.

2. **carcontroller branches on `CC.enabled`** to drive the ADAS DRV ECU's UDS state. While engaged: emit longitudinal commands and the 1 Hz tester-present-to-`0x730`, as today. On the engage→disengage transition (single frame, edge-triggered): emit `UDS 0x28 0x00 (CommunicationControl: enableRxAndTx)` to `0x730`, then stop emitting tester-present. While disengaged: emit nothing on the longitudinal addresses or `0x730`. On the disengage→engage transition: resume today's behavior — the boot-time UDS disable sequence is already in effect from boot and only needs the tester-present keep-alive to continue, but if we emit the explicit `UDS 0x28 0x03` to redisable, that's symmetric. Phase 1 trace work confirms whether the redisable is necessary or whether the ECU stays in its previous comm-control state.

A new param `DynamicRadarHandoffEnabled`, sub-toggled under Alpha Longitudinal in the Hyundai settings layout, gates whether either mechanism is active. When off, the safety_param flag bit is not set and carcontroller behaves as today. The toggle is visible only when `CP.flags & HyundaiFlags.CANFD_LKA_STEERING` and Alpha Long is enabled.

## Components

### 5.1 Trace capture & analysis

**Path:** `selfdrive/debug/car/hyundai_canfd_handoff_traces/`

- `capture.py` — extends the pandad-bypass pattern from `selfdrive/debug/car/hyundai_enable_radar_points.py:106-107`. Puts the panda in `elm327` passive-listen safety mode, logs raw CAN-FD on buses 0/1/2 to a timestamped `.log` file. Two procedures supported: (a) stock baseline — ignition on, stock SCC engage at speed, set/resume cycles, brake-disengage, re-engage, ignition off; (b) sunnypilot capture with the existing alpha-long behavior — same drive flow, observe the UDS handshake at boot and the tester-present cadence to `0x730`.
- `analyze.py` — offline. Parses a capture, extracts:
  - The boot-time UDS sequence to `0x730` — exact session type, communication control sub-function, tester-present cadence.
  - Stock ADAS DRV ECU message sequences across each engage→disengage transition — counter rollover, sequence-byte continuity, checksum recomputation, which addresses go quiet vs. continue under stock disengaged.
  - The ADAS DRV ECU's responses to the boot-time disable (if any).
- `findings.md` — committed alongside captured artifacts. Documents:
  1. The exact UDS sub-function sequence sunnypilot sends today.
  2. The expected ADAS DRV ECU behavior under stock operation.
  3. Whether stopping tester-present alone is sufficient for the ECU to self-recover (session timeout), or whether the explicit `UDS 0x28 0x00` is required.
  4. Whether the redisable on re-engage requires the full UDS sequence or just resumed tester-present.

### 5.2 Hyundai CAN-FD safety code

**Path:** `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h`

Today's structure:

- `hyundai_canfd_hooks` (lines 384-391) defines `init`, `rx`, `tx`, `get_counter`, `get_checksum`, `compute_checksum`. **No `fwd` hook is defined.** Forwarding goes through the framework's default `safety_fwd_hook` at `opendbc/safety/safety.h:260-281`.
- The default `safety_fwd_hook` blocks any address that's in the static TX allow-list with `check_relay=true` AND is destined for the bus where the addr appears. This is how alpha-long currently blocks stock SCC frames from reaching the powertrain side.

Changes:

- **New flag bit:** `HYUNDAI_PARAM_CANFD_DYNAMIC_HANDOFF = 1024` (2¹⁰), defined inline in `hyundai_canfd_init()` alongside the existing `HYUNDAI_PARAM_CANFD_LKA_STEERING_ALT` and `HYUNDAI_PARAM_CANFD_ALT_BUTTONS` constants. Corresponding entry in `HyundaiSafetyFlags` in `opendbc_repo/opendbc/car/hyundai/values.py` (existing enum at lines 60-71).
- **New file-scope bool:** `hyundai_canfd_dynamic_handoff`, set in `hyundai_canfd_init()` via `GET_FLAG(param, HYUNDAI_PARAM_CANFD_DYNAMIC_HANDOFF)`, mirroring the pattern at lines 277-278.
- **Mark longitudinal TX entries with `disable_static_blocking=true`** in the HDA II long TX list (`HYUNDAI_CANFD_LKA_STEERING_LONG_TX_MSGS` at lines 239-250) — specifically the SCC_CONTROL entry and the ADRV addresses (0x51, 0x160, 0x1EA, 0x200, 0x345, 0x1DA). The framework explicitly supports this pattern (`opendbc/safety/declarations.h:89`: *"if true, static blocking is disabled so safety mode can dynamically handle it (e.g. selective AEB pass-through)"*). This change is conditional only in that the new `fwd` hook (below) honors it differently when the flag is set; existing TX-list semantics for other safety modes are unchanged.
- **Add a `fwd` hook** to `hyundai_canfd_hooks`. Signature per `opendbc/safety/declarations.h:213`: `bool fwd_hook(int bus_num, int addr)` — returns true to block forwarding, false to allow. When `hyundai_canfd_dynamic_handoff` is true: for the longitudinal addresses, return `controls_allowed` (block forwarding when openpilot is the authority; allow forwarding when openpilot is disengaged). When `hyundai_canfd_dynamic_handoff` is false: return false for all addresses (no change from today, since the static blocking still applies).
- **Extend the TX hook** at `hyundai_canfd_tx_hook` (lines 143-225). When `hyundai_canfd_dynamic_handoff` is true and `!controls_allowed`, reject TX of SCC_CONTROL and the ADRV control addresses. When the flag is false, no change.
- **Do not gate UDS frames on `controls_allowed` in the safety code.** Addresses `0x730` (and `0x7D0` on non-HDA-II platforms) carry both the engaged-state tester-present and the disengaged-state deinit; the safety code must allow either at any time. carcontroller controls timing.

**Tests:** `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py` (existing file) extended with:

- A test that with the flag set and `controls_allowed=true`, the TX hook accepts SCC_CONTROL and ADRV frames.
- A test that with the flag set and `controls_allowed=false`, the TX hook rejects those frames.
- A test that with the flag set and `controls_allowed=false`, the forwarding hook allows stock SCC/FCA frames through (no blocking).
- A test that with the flag set and `controls_allowed=true`, the forwarding hook blocks stock SCC/FCA frames.
- A test that `0x730` TX is allowed in both states (the UDS path is not gated).
- A test that with the flag unset, behavior is identical to today — regression coverage for existing alpha-long users.
- A replay test that consumes a Phase 1 fixture trace and verifies safety decisions match what the trace shows the stock car doing in passthrough mode.

### 5.3 CarParams wiring

**Path:** `opendbc_repo/opendbc/car/hyundai/interface.py`

At CarParams init, when **all** of the following hold:

- `CP.flags & HyundaiFlags.CANFD_LKA_STEERING` is set (i.e. HDA II detected via fingerprint at `interface.py:58`).
- `CP.flags & HyundaiFlags.CANFD_NO_RADAR_DISABLE` is **not** set.
- `CP.flags & HyundaiFlags.CANFD_CAMERA_SCC` is **not** set.
- `params.get_bool("DynamicRadarHandoffEnabled")` is true.
- `params.get_bool("AlphaLongitudinalEnabled")` is true (i.e. openpilotLongitudinalControl will be enabled).

OR-in `HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF.value` to the Hyundai CAN-FD safety_param built around `interface.py:81-84`. Under any condition unmet, the bit is clear and safety behavior is unchanged.

pandad itself requires no changes. The existing flow at `selfdrive/pandad/panda_safety.cc:60-79` reads `safety_configs` from CarParams and calls `set_safety_model(model, safety_param)` at boot — the new flag bit rides through this existing path. The `safety_configured_` one-shot latch in `panda_safety.cc:14` stays.

**Tests:** `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py` extended with:

- With all conditions met, the constructed safety_param has the new bit set.
- With each individual condition unmet (no HDA II flag; with `CANFD_NO_RADAR_DISABLE`; with `CANFD_CAMERA_SCC`; param off; alpha long off), the bit is clear.

### 5.4 carcontroller + UI

**Paths:** `opendbc_repo/opendbc/car/hyundai/carcontroller.py`, `selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py`

**carcontroller — longitudinal command gating.** In `create_canfd_msgs()` (called from `carcontroller.py:122`), when `DynamicRadarHandoffEnabled` is true and `CC.enabled` is false, skip emitting SCC_CONTROL and the ADRV control frames. Today's behavior preserved when the param is false.

**carcontroller — tester-present gating.** The block at `carcontroller.py:107-114` currently emits tester-present-to-`0x730` (on HDA II) at 1 Hz whenever `openpilotLongitudinalControl` is true and not Camera SCC. Extend the condition: when `DynamicRadarHandoffEnabled` is true, additionally require `CC.enabled` to be true. When the param is false, no change.

**carcontroller — UDS deinit on disengage transition.** Track `prev_enabled` across `carcontroller.update()` calls. On the transition from `prev_enabled=true` to `CC.enabled=false`, when `DynamicRadarHandoffEnabled` is true, emit:

- `UDS 0x28 0x00 (CommunicationControl: enableRxAndTx)` to `0x730` on `self.CAN.ECAN`, with suppress response. Single frame, edge-triggered, not repeated.

A helper `make_communication_control_msg(addr, bus, sub_function, suppress_response=True)` is added to `opendbc_repo/opendbc/car/__init__.py` alongside the existing `make_tester_present_msg`, since the UDS service is generic enough to belong with the common helpers.

**carcontroller — disengage→engage redisable.** On the transition from `prev_enabled=false` to `CC.enabled=true`, when `DynamicRadarHandoffEnabled` is true, the carcontroller emits no special frame in v1. Phase 1 trace findings determine whether the ECU stays in its previous state (no redisable needed because tester-present alone keeps it down) or whether the explicit `UDS 0x28 0x03` is required. If required, this is a one-line follow-up; the spec defers the decision.

**UI sub-toggle.** New entry "Dynamic Radar Handoff" in `selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py`. Visible only when (a) Alpha Longitudinal is enabled and (b) `CP.flags & HyundaiFlags.CANFD_LKA_STEERING` and (c) not `CP.flags & HyundaiFlags.CANFD_NO_RADAR_DISABLE` and not `CP.flags & HyundaiFlags.CANFD_CAMERA_SCC`. Tooltip: *"Restores stock SCC and AEB when sunnypilot is disengaged. AEB may not re-arm reliably after disengagement; stock behavior is hardware-dependent. Out of scope: stock AEB is not preserved while sunnypilot is engaged."*

**Tests:**

- carcontroller: `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py` extended to assert:
  - With `DynamicRadarHandoffEnabled=true` and `CC.enabled=false`, no SCC_CONTROL or ADRV frame is emitted.
  - With `DynamicRadarHandoffEnabled=true` and `CC.enabled=false`, no tester-present-to-`0x730` is emitted.
  - With `DynamicRadarHandoffEnabled=true`, on the engage→disengage edge, exactly one `UDS 0x28 0x00` frame to `0x730` is emitted, and not repeated on subsequent disengaged cycles.
  - With `DynamicRadarHandoffEnabled=false`, all of today's behavior is preserved.
- UI: **create** a new test at `selfdrive/ui/tests/test_hyundai_brand_layout.py` (no equivalent exists today). Asserts the sub-toggle is hidden under each unmet visibility condition and visible when all are met.

## Data flow

### Boot

Sunnypilot starts, alpha long enabled, dynamic handoff enabled, GV70 Electrified (or equivalent HDA II platform). At CarParams init, `HYUNDAI_PARAM_CANFD_DYNAMIC_HANDOFF` is OR'd into the Hyundai CAN-FD safety_param. pandad applies the safety model at `ControlsReady`. Safety code in panda reads the flag bit, sets `hyundai_canfd_dynamic_handoff = true`. `controls_allowed` defaults to false. Safe state: `STOCK_PASSTHROUGH` — longitudinal TX is rejected at the safety layer, stock SCC/FCA frames forward freely.

carcontroller emits no longitudinal frames (CC.enabled is false at boot). The boot-time UDS sequence sent by carcontroller today — `UDS 0x10 0x03` extended session and `UDS 0x28 0x03` disable — does **not** happen under dynamic handoff because the carcontroller change gates them on `CC.enabled`. The ADAS DRV ECU stays in default session, comm enabled, AEB armed. This is the desired state at boot.

(Note: this assumes the boot-time disable sequence is gated by `openpilotLongitudinalControl AND CC.enabled` under dynamic handoff. If the boot disable is not actually emitted by carcontroller but rather by some other path — Phase 1 trace work confirms — this section is reviewed.)

### Engage transition (disengaged → engaged)

```
selfdriveState.enabled flips true
  │
  ├─ pandad's 10Hz heartbeat tick (next ≤100ms): pandad.cc:401-407
  │     └─ send_heartbeat(engaged=true, ...) → panda firmware 0xf3
  │           └─ opendbc/safety/safety.h sets controls_allowed = true
  │                 (hyundai_canfd TX hook now accepts SCC_CONTROL/ADRV;
  │                  fwd hook now blocks stock SCC/FCA forwarding)
  │
  └─ carcontroller's 100Hz tick (next ≤10ms): CC.enabled flips true
        ├─ begins synthesizing SCC_CONTROL and ADRV frames
        ├─ resumes 1Hz tester-present-to-0x730
        └─ (Phase 1 finding) if redisable required: emit UDS 0x28 0x03 once
```

Race window: same 100ms-bounded window as before. carcontroller may emit frames that the safety code rejects until the heartbeat lands; safety code may accept frames carcontroller hasn't built yet. Both harmless.

### Disengage transition (engaged → disengaged)

```
selfdriveState.enabled flips false
  │
  ├─ pandad's 10Hz tick (next ≤100ms):
  │     └─ send_heartbeat(engaged=false) → controls_allowed = false
  │           (TX hook now rejects SCC_CONTROL/ADRV;
  │            fwd hook now allows stock SCC/FCA forwarding)
  │
  └─ carcontroller's 100Hz tick (next ≤10ms): CC.enabled flips false
        ├─ stops emitting SCC_CONTROL and ADRV frames
        ├─ stops emitting tester-present-to-0x730
        └─ emits single UDS 0x28 0x00 (enableRxAndTx) to 0x730  [DEINIT]
```

The single deinit frame is the load-bearing piece. It tells the ADAS DRV ECU "you may now communicate normally" — undoing the boot-time `UDS 0x28 0x03`. The ECU should then resume its normal SCC/AEB operation.

### Steady state — disengaged

carcontroller emits no longitudinal frames, no tester-present, no UDS frames. Safety code allows stock SCC/FCA frames to forward between buses unmodified. The ADAS DRV ECU's previously-locked-out frames now reach the powertrain side. Stock SCC and AEB function as on an unmodified car.

### Steady state — engaged

Today's alpha-long behavior. carcontroller emits SCC_CONTROL, ADRV frames, and 1 Hz tester-present-to-`0x730`. Safety code allows openpilot TX, blocks stock SCC/FCA forwarding.

### What does NOT flow

- No new USB control message between pandad and the panda.
- No new pandad-side state machine.
- No polling, no acks, no retries for the deinit frame. It's fire-and-forget. If lost, the next engage→disengage cycle re-issues it. Worst case: AEB stays offline until the next disengage that succeeds.

## Error handling

### Deinit frame is lost in transit

Fire-and-forget. Next disengage transition emits another deinit. In practice the panda's USB-to-CAN path is reliable enough that this is rare; we accept the worst-case scenario of "AEB stays offline until the next disengage" without adding retry/ack logic.

### ADAS DRV ECU does not respond to deinit

No detection. The ECU's behavior is observed in Phase 1; if its firmware silently ignores `UDS 0x28 0x00` outside of a specific session context, that's flagged in `findings.md` and the spec needs revision to send the session-control frame first. v1 sends only the bare CommunicationControl frame; if Phase 1 reveals this is insufficient, a one-line addition prepends `UDS 0x10 0x01` (default session).

### Radar latches a DTC across a transition

No detection, no reaction. User sees the stock dashboard warning. AEB is offline for the remainder of the drive; sunnypilot continues operating normally. Documented in the UI sub-toggle tooltip. Detecting the fault would require parsing diagnostic frames whose format we have not characterized; acting on detection gains nothing.

### Heartbeat-derived state drifts from controlsd's engaged state

`opendbc/safety/safety.h:74` already tracks `heartbeat_engaged_mismatches`. Existing timeout logic forces `controls_allowed = false` on stuck heartbeats. The dynamic handoff inherits this for free: a stuck heartbeat fails closed to `STOCK_PASSTHROUGH`, which is the safe default.

### Engage signal arrives during pandad startup or before `ControlsReady`

Existing `ControlsReady` gate at `selfdrive/pandad/panda_safety.cc:52-55` is preserved. Until ControlsReady, no Hyundai CAN-FD safety model is applied. `controls_allowed` defaults to false. Carcontroller does not run before this point. Safe default in effect.

### What is NOT handled by design

- No active radar health monitoring or DTC parsing.
- No fallback to vision-only on detected fault.
- No persistent "this car is broken" flag.
- No UDS ECU Reset panic-recovery path in v1.
- No claim about non-HDA-II HKG CAN-FD platforms.

## Testing strategy

### Phase 1 — Trace capture & analysis

- `capture.py` has no automated test; produces data, data is the deliverable.
- `analyze.py` has unit tests at `selfdrive/debug/car/hyundai_canfd_handoff_traces/test_analyze.py`. Fixture `.log` files committed; outputs match annotated expectations.
- Acceptance: `findings.md` answers the four questions in 5.1, signed off by the user.

### Phase 2 — Hyundai CAN-FD safety code

- `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py` extended per 5.2 test list.
- Coverage gap: the safety tests do not simulate the ADAS DRV ECU's response to the UDS deinit. That is HIL territory.

### Phase 3 — CarParams wiring

- `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py` extended per 5.3 test list.

### Phase 4 — carcontroller + UI

- carcontroller: `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py` extended per 5.4 test list.
- UI: **create** `selfdrive/ui/tests/test_hyundai_brand_layout.py` (no precedent — this phase introduces the first vehicle-brand layout visibility test).

### Phase 5 — Hardware-in-the-loop acceptance

The end-to-end claim — that the safety code's `controls_allowed`-gated TX/forwarding plus carcontroller's UDS deinit frame allows the stock ADAS DRV ECU to remain healthy across engage/disengage cycles — cannot be validated in CI. Requires a real HDA II HKG CAN-FD car (GV70 Electrified for the first validation) and a scan tool.

HIL acceptance criteria:

1. **Baseline drive.** `DynamicRadarHandoffEnabled=false`, `AlphaLongitudinalEnabled=false` (stock SCC). ≥15 minutes, ≥5 stock engage/disengage cycles. End: scan-tool readout of the full module list shows zero new SCC, FCA, or ADAS DRV DTCs since start of drive.

2. **Three test drives** on consecutive ignition cycles (separate ignition-on/off events; not required to be consecutive days). Each: ≥15 minutes, ≥5 sunnypilot engage/disengage cycles. End: scan-tool readout shows zero new SCC, FCA, or ADAS DRV DTCs since start of drive. Intermittent codes that self-clear before readout are recorded but not failures.

3. **AEB armed verification.** During each test drive, after at least one disengage following an engage, the driver visually confirms the stock collision-warning indicator on the cluster is in its normal armed state.

Pass: all four drives produce zero new DTCs and a normal cluster indicator post-disengage. Sub-toggle moves from developer panel to standard Vehicle → Hyundai settings.

Fail: any drive latches a DTC or shows a cluster fault. Findings inform a follow-up — most likely a `UDS 0x10 0x01` prepend or a Phase 1-discovered alternate sequence. Sub-toggle stays behind the developer panel.

## Rollout

Five phases, each independently mergeable as a no-op for existing users until the next phase activates it:

1. **Phase 1 — Trace capture & analysis.** Independently useful. Other HKG longitudinal work can use the tooling. Mergeable on its own.
2. **Phase 2 — Safety code.** Depends on Phase 1's `findings.md` for fixture data. Without Phase 3, no platform sets the new flag bit; existing alpha-long users see no change.
3. **Phase 3 — CarParams wiring.** Depends on Phase 2. Adds the param read and OR-in. Without Phase 4, the param doesn't exist in the UI; feature stays off; existing users see no change.
4. **Phase 4 — carcontroller + UI.** Depends on Phases 2 and 3. After Phase 4, the feature is wired end-to-end. Sub-toggle lives in the developer panel until Phase 5 passes.
5. **Phase 5 — HIL acceptance.** Manual, on a real HDA II car (GV70 Electrified for the first round). On pass: sub-toggle moves to Vehicle → Hyundai settings. On fail: stays behind developer panel; follow-up spec.

## Open questions deferred

- **Generalization to other HDA II platforms** beyond the first validated car (likely GV70 Electrified). The runtime gate (`CANFD_LKA_STEERING` flag) already covers them by design; per-platform HIL validation determines when each moves out of the developer panel.
- **Generalization to non-HDA-II HKG CAN-FD platforms** (those that use `0x7D0` radar directly). The framework is platform-gated; extension is a follow-up spec.
- **Whether `UDS 0x10 0x01` (default session) needs to precede the `UDS 0x28 0x00` deinit.** Phase 1 trace work and Phase 5 HIL drives answer this. If yes, a one-line addition in 5.4.
- **Whether a redisable on re-engage is required** or whether tester-present alone is sufficient to maintain the ECU in its disabled state. Phase 1 findings determine this.
- **Whether `findings.md` is a hard prerequisite for Phase 2** or whether Phase 2 can land behind the developer panel with stub fixtures and absorb Phase 1 findings as they come.

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
- **Do not gate UDS frames on `controls_allowed` in the safety code.** Addresses `0x730` (and `0x7D0` on non-HDA-II platforms) carry both the engaged-state tester-present and the engage/disengage edge sequences; the safety code must allow them at any time. carcontroller controls timing.
- **Widen the 0x730 UDS allowlist under the handoff bit.** The pre-existing tester-present-only allowlist at `hyundai_canfd_tx_hook` would otherwise drop every non-tester-present frame the carcontroller emits in this feature. When `hyundai_canfd_dynamic_handoff` is true, additionally accept the four exact no-suppress-response payloads the carcontroller fires on the engage/disengage edges: `02 10 03 00` (extendedDiagnosticSession), `03 28 03 01` (disableRxAndTx), `03 28 00 01` (enableRxAndTx), `02 10 01 00` (defaultSession). The suppress-bit is intentionally clear so the ECU emits positive acks (`02 50 03`, `02 68 03`, `02 68 00`, `02 50 01`) and NRCs (`03 7F 10 <code>`, `03 7F 28 <code>`) on `0x738` — these land in route/cabana logs and provide free Phase 1 / post-incident diagnostic visibility. The tester-present pattern (which fires at 1 Hz) keeps its suppress bit. All other 0x730 payloads remain rejected.

**Tests:** `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py` (existing file) extended with:

- A test that with the flag set and `controls_allowed=true`, the TX hook accepts SCC_CONTROL and ADRV frames.
- A test that with the flag set and `controls_allowed=false`, the TX hook rejects those frames.
- A test that with the flag set and `controls_allowed=false`, the forwarding hook allows stock SCC/FCA frames through (no blocking).
- A test that with the flag set and `controls_allowed=true`, the forwarding hook blocks stock SCC/FCA frames.
- A test that with the flag set, each of the four handoff no-suppress payloads on `0x730` (`02 10 03 00`, `03 28 03 01`, `03 28 00 01`, `02 10 01 00`) is accepted regardless of `controls_allowed`.
- A test that with the flag set, other 0x730 payloads (the same four with the suppress bit set, ECU reset, wrong communicationType) are still rejected.
- A test that with the flag unset, none of the four handoff payloads is accepted on `0x730` (strict tester-present-only).
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

OR-in `HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF.value` to the Hyundai CAN-FD safety_param (done by `_initialize_dynamic_radar_handoff` in `opendbc/sunnypilot/car/interfaces.py`, fired from `setup_interfaces` which runs from `get_car` before `CarInterface.init`). Under any condition unmet, the bit is clear and safety behavior is unchanged.

**Boot-time `disable_ecu` gating.** `CarInterface.init` at `interface.py:230-241` calls `disable_ecu(0x730)` whenever long mode is on. Under dynamic handoff this must be skipped — the carcontroller re-applies the disable on each engage edge instead, so the boot path must leave the ADAS DRV ECU in defaultSession with comm armed. Add an `if (CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF) skip` guard in `init`. By the time `init` runs, `setup_interfaces` has already set the bit, so reading from `CP.safetyConfigs[-1].safetyParam` is correct.

pandad itself requires no changes. The existing flow at `selfdrive/pandad/panda_safety.cc:60-79` reads `safety_configs` from CarParams and calls `set_safety_model(model, safety_param)` at boot — the new flag bit rides through this existing path. The `safety_configured_` one-shot latch in `panda_safety.cc:14` stays.

**Tests:** `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py` extended with:

- With all conditions met, the constructed safety_param has the new bit set.
- With each individual condition unmet (no HDA II flag; with `CANFD_NO_RADAR_DISABLE`; with `CANFD_CAMERA_SCC`; param off; alpha long off), the bit is clear.

### 5.4 carcontroller + UI

**Paths:** `opendbc_repo/opendbc/car/hyundai/carcontroller.py`, `selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py`

**carcontroller — longitudinal command gating.** In `create_canfd_msgs()` (called from `carcontroller.py:122`), when `DynamicRadarHandoffEnabled` is true and `CC.enabled` is false, skip emitting SCC_CONTROL and the ADRV control frames. Today's behavior preserved when the param is false.

**carcontroller — tester-present gating.** The block at `carcontroller.py:107-114` currently emits tester-present-to-`0x730` (on HDA II) at 1 Hz whenever `openpilotLongitudinalControl` is true and not Camera SCC. Extend the condition: when `DynamicRadarHandoffEnabled` is true, additionally require `CC.enabled` to be true. When the param is false, no change.

**carcontroller — UDS deinit on disengage transition.** Track `prev_enabled` across `carcontroller.update()` calls. On the transition from `prev_enabled=true` to `CC.enabled=false`, when `DynamicRadarHandoffEnabled` is true, emit two **non-suppress-response** frames to `0x730` on `self.CAN.ECAN`:

- `UDS 0x28 0x00 (CommunicationControl: enableRxAndTx)` — re-enables Rx/Tx in the current (extended) session.
- `UDS 0x10 0x01 (DiagnosticSessionControl: defaultSession)` — drops the ECU back to defaultSession, which on Hyundai also resets any residual CommunicationControl state and arms stock SCC/AEB immediately (no ~5 s S3 timeout wait).

Either frame alone restores comm; sending both gives a redundant fast-recovery path. Both are edge-triggered (single-shot per disengage), not repeated. Suppress-response is intentionally NOT set: the ECU's positive acks (`02 68 00`, `02 50 01`) and any NRCs (`03 7F 28 …`, `03 7F 10 …`) land on `0x738` in route/cabana logs — free diagnostic visibility with negligible bus-load cost.

Helpers `make_communication_control_msg` and `make_diagnostic_session_control_msg` are added to `opendbc_repo/opendbc/car/__init__.py` alongside the existing `make_tester_present_msg`.

**carcontroller — disengage→engage redisable.** Boot-time `disable_ecu` is skipped under dynamic handoff (see 5.3); the ADAS DRV ECU is alive at boot and during every disengaged window. On the transition from `prev_enabled=false` to `CC.enabled=true`, when `DynamicRadarHandoffEnabled` is true, the carcontroller emits two non-suppress-response frames to `0x730` to re-silence it:

- `UDS 0x10 0x03 (DiagnosticSessionControl: extendedDiagnosticSession)` — enter the diagnostic session in which CommunicationControl persists.
- `UDS 0x28 0x03 (CommunicationControl: disableRxAndTx)` — silence the ECU.

The 1 Hz tester-present that resumes on the same frame keeps the extended session alive (S3 ≈ 5 s). Both frames are edge-triggered, not repeated. Same non-suppress rationale as the disengage pair: positive acks (`02 50 03`, `02 68 03`) and NRCs land in trace logs. The 1 Hz tester-present keeps its suppress bit set — it fires too often for its acks to be useful, and would otherwise add 1 Hz of bus chatter for the whole engaged window.

**UI sub-toggle.** New entry "Dynamic Radar Handoff" in `selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py`. Visible only when (a) Alpha Longitudinal is enabled and (b) `CP.flags & HyundaiFlags.CANFD_LKA_STEERING` and (c) not `CP.flags & HyundaiFlags.CANFD_NO_RADAR_DISABLE` and not `CP.flags & HyundaiFlags.CANFD_CAMERA_SCC`. Tooltip: *"Restores stock SCC and AEB when sunnypilot is disengaged. AEB may not re-arm reliably after disengagement; stock behavior is hardware-dependent. Out of scope: stock AEB is not preserved while sunnypilot is engaged."*

**Tests:**

- carcontroller: `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py` extended to assert:
  - With `DynamicRadarHandoffEnabled=true` and `CC.enabled=false`, no SCC_CONTROL or ADRV frame is emitted.
  - With `DynamicRadarHandoffEnabled=true` and `CC.enabled=false`, no tester-present-to-`0x730` is emitted.
  - With `DynamicRadarHandoffEnabled=true`, on the engage→disengage edge, exactly one `UDS 0x28 0x00` and one `UDS 0x10 0x01` frame to `0x730` is emitted, and neither is repeated on subsequent disengaged cycles.
  - With `DynamicRadarHandoffEnabled=true`, on the disengage→engage edge, exactly one `UDS 0x10 0x03` and one `UDS 0x28 0x03` frame to `0x730` is emitted, and neither is repeated on subsequent engaged cycles.
  - With `DynamicRadarHandoffEnabled=true`, the boot-time `CarInterface.init` `disable_ecu` call to `0x730` is skipped.
  - With `DynamicRadarHandoffEnabled=false`, all of today's behavior is preserved.
- UI: **create** a new test at `selfdrive/ui/tests/test_hyundai_brand_layout.py` (no equivalent exists today). Asserts the sub-toggle is hidden under each unmet visibility condition and visible when all are met.

## Data flow

### Boot

Sunnypilot starts, alpha long enabled, dynamic handoff enabled, GV70 Electrified (or equivalent HDA II platform). At CarParams init, `HYUNDAI_PARAM_CANFD_DYNAMIC_HANDOFF` is OR'd into the Hyundai CAN-FD safety_param. pandad applies the safety model at `ControlsReady`. Safety code in panda reads the flag bit, sets `hyundai_canfd_dynamic_handoff = true`. `controls_allowed` defaults to false. Safe state: `STOCK_PASSTHROUGH` — longitudinal TX is rejected at the safety layer, stock SCC/FCA frames forward freely.

carcontroller emits no longitudinal frames (CC.enabled is false at boot). The boot-time UDS sequence today lives in `CarInterface.init` at `interface.py:230-241` — `disable_ecu(addr=0x730, com_cont_req=\x28\x83\x01)` which sends `UDS 0x10 0x03` then `UDS 0x28 0x83 0x01`. Under dynamic handoff, that call is **gated on the `CANFD_DYNAMIC_HANDOFF` safety bit and skipped**. The ADAS DRV ECU stays in default session, comm enabled, AEB armed. This is the desired state at boot.

(Earlier drafts assumed the boot disable lived in carcontroller — it does not. The fix is in `CarInterface.init`, not in carcontroller; carcontroller emits the re-disable on the disengage→engage edge instead.)

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
        └─ emits UDS 0x10 0x03 (extendedDiagnosticSession) + UDS 0x28 0x83 0x01 (disableRxAndTx)  [REDISABLE]
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
        └─ emits UDS 0x28 0x00 (enableRxAndTx) + UDS 0x10 0x01 (defaultSession) to 0x730  [DEINIT]
```

The deinit pair is the load-bearing piece. `0x28 0x00` tells the ADAS DRV ECU "you may now communicate normally" — undoing the engage-edge `UDS 0x28 0x83 0x01`. `0x10 0x01` then drops the ECU back to defaultSession, which on Hyundai resets any residual CommunicationControl state and arms stock SCC/AEB immediately (no ~5 s S3 timeout wait). Either frame alone restores comm; sending both is the redundant fast-recovery path.

### Steady state — disengaged

carcontroller emits no longitudinal frames, no tester-present, no UDS frames. Safety code allows stock SCC/FCA frames to forward between buses unmodified. The ADAS DRV ECU's previously-locked-out frames now reach the powertrain side. Stock SCC and AEB function as on an unmodified car.

### Steady state — engaged

Today's alpha-long behavior. carcontroller emits SCC_CONTROL, ADRV frames, and 1 Hz tester-present-to-`0x730`. Safety code allows openpilot TX, blocks stock SCC/FCA forwarding.

### What does NOT flow

- No new USB control message between pandad and the panda.
- No new pandad-side state machine.
- No polling, no acks, no retries for any edge-triggered frame. All four (engage `0x10 0x03` + `0x28 0x83 0x01`, disengage `0x28 0x00` + `0x10 0x01`) are fire-and-forget. The two-frame redundancy on each edge mitigates single-frame loss; if both engage frames are lost the ECU is never re-silenced (stock + sunnypilot SCC will fight on next engage), if both disengage frames are lost the ECU stays disabled until the next disengage that succeeds.

## Error handling

### Edge frame is lost in transit

Fire-and-forget with two-frame redundancy on each edge:

- **Disengage edge.** `0x28 0x00` and `0x10 0x01` are independently sufficient (either restores comm). Both being lost is the only failure case, in which the ECU stays disabled until the next disengage that succeeds.
- **Engage edge.** `0x10 0x03` and `0x28 0x83 0x01` are sequentially dependent — losing the first means the second has no effect (CommControl doesn't persist in defaultSession). Losing both, or just the first, means the ECU is never re-silenced on this engage cycle and the stock ADAS DRV will fight openpilot's SCC_CONTROL. The fwd hook still blocks stock SCC forwarding (`controls_allowed=true`), so the user-visible failure is at most a stock SCC frame the panda blocks before it reaches the vehicle CAN — not a control-fight on the bus.

In practice the panda's USB-to-CAN path is reliable enough that the dropped-edge case is rare; we accept the worst-case scenarios above without adding retry/ack logic.

### ADAS DRV ECU does not respond to a UDS frame

No detection. v1 sends suppress-response so positive acks aren't expected; NRCs are still emitted by the ECU (the suppress bit only mutes positive responses) and visible in a candump trace if the ECU rejects. The engage and disengage sequences are designed redundantly per above, so a single-frame rejection still leaves a working path.

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

Fail: any drive latches a DTC or shows a cluster fault. Findings inform a follow-up — most likely a Phase 1-discovered alternate session sequence, an `0x11 (ECUReset)` panic-recovery path, or a sub-function tweak. Sub-toggle stays behind the developer panel.

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
- ~~**Whether `UDS 0x10 0x01` (default session) needs to precede the `UDS 0x28 0x00` deinit.**~~ **Resolved before Phase 1 by deriving from ISO 14229-1 semantics: yes, send `0x10 0x01` — but as the second frame in the deinit pair, not as a prepend.** On Hyundai, CommControl persists only as long as the extended session does (`opendbc/car/disable_ecu.py:12`); dropping back to defaultSession also resets any residual CommControl. Sending both makes each frame independently sufficient. See 5.4.
- ~~**Whether a redisable on re-engage is required** or whether tester-present alone is sufficient.~~ **Resolved: required.** With the boot disable skipped under handoff (5.3), the ECU is in defaultSession at every engage. Tester-present in defaultSession does not silence the ECU; an explicit `0x10 0x03` + `0x28 0x83 0x01` pair on the engage edge is mandatory. See 5.4.
- **Whether `findings.md` is a hard prerequisite for Phase 2** or whether Phase 2 can land behind the developer panel with stub fixtures and absorb Phase 1 findings as they come.

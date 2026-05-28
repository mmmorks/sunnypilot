# HKG CAN-FD HDA II Handoff Trace Findings

Empirical record of CAN-FD bus behavior on real HDA II vehicles across SCC
engage/disengage cycles, both stock and with sunnypilot alpha longitudinal
enabled. Phase 2 safety-code design and Phase 4 carcontroller behavior reference
this document.

## Procedure

### Stock baseline run

1. Vehicle off; ensure openpilot is not running (`sudo systemctl stop pandad` or unplug C3).
2. Connect panda over USB to a laptop running `capture.py`.
3. Ignition on, engine off (key in run position).
4. Run `python capture.py --out fixtures/stock_baseline_<car>_<date>.log`.
5. Drive procedure (closed road or test track):
   - Engage stock SCC at 50 km/h.
   - Press SET twice to confirm engagement.
   - Press RES three times to increment set speed.
   - Brake to disengage.
   - Re-engage SCC.
   - Hold for 30 s.
   - Disengage via stalk cancel.
   - Repeat 5 times.
   - Ignition off.
6. Move the resulting .log file into this directory's `fixtures/`.

### Sunnypilot alpha-long run (existing behavior)

Same procedure, but with openpilot running in alpha-long mode (no dynamic
handoff toggle yet). Capture the boot-time UDS sequence sunnypilot performs.

## Findings template

For each captured fixture, document:

- **Capture date / platform / VIN suffix.**
- **Boot-time UDS sequence to 0x730 (verbatim hex):**
  - Session control: e.g. `0x10 0x03` (extendedDiagnosticSession)
  - Communication control: e.g. `0x28 0x03 0x01` (subFunction + communicationType)
  - Tester-present cadence: hz, with/without suppress-response
- **What 0x730 emits between transitions (counter/sequence behavior).**
- **Whether stopping tester-present alone is sufficient for the ECU to self-recover within 5 seconds (timeout).**
- **Whether a redisable on re-engage is required, or whether tester-present resumption is sufficient.**
- **Whether `UDS 0x28 0x00` (enableRxAndTx) alone restores the ECU, or whether a preceding `UDS 0x10 0x01` (defaultSession) is required.**
- **DTC scan-tool readout at end of drive (full module list).**

## Captures

(Populate after Phase 1 HIL data collection.)

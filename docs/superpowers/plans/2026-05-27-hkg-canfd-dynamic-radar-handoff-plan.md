# HKG CAN-FD HDA II Dynamic Radar Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Spec:** `docs/superpowers/specs/2026-05-27-hkg-canfd-dynamic-radar-handoff-design.md`

> **Post-implementation amendment (2026-05-29).** The original plan called for a single `0x28 0x00` disengage frame and no engage-edge work. Code review found that (a) the panda's existing 0x730 allowlist accepts only tester-present, so the `0x28 0x00` was being silently dropped, and (b) the boot disable lives in `CarInterface.init` — not the carcontroller — so gating it on `CC.enabled` never had effect; the ECU was silenced at boot and only stayed silenced for ~5 s before S3 expired. The implementation now:
>
> 1. Widens the safety allowlist on `0x730` to permit four exact **non-suppress-response** patterns under the handoff bit: `0x10 0x03 0x00` (extendedDiagnosticSession), `0x28 0x03 0x01` (disableRxAndTx), `0x28 0x00 0x01` (enableRxAndTx), `0x10 0x01 0x00` (defaultSession). Suppress-response is intentionally NOT set on edge frames so the ECU's positive acks and NRCs land on `0x738` in route/cabana logs for free diagnostic visibility. The 1 Hz tester-present keeps its suppress bit.
> 2. Skips the boot-time `disable_ecu(0x730)` call in `CarInterface.init` when the `CANFD_DYNAMIC_HANDOFF` safety bit is set.
> 3. Emits the `0x10 0x03` + `0x28 0x83 0x01` pair on the disengage→engage edge to re-silence the ECU (because the boot disable was skipped).
> 4. Emits the `0x28 0x00` + `0x10 0x01` pair on the engage→disengage edge (the `0x10 0x01` is a redundant fast-recovery path: it drops the ECU to defaultSession immediately rather than waiting ~5 s for S3 to time out).
> 5. Adds a `make_diagnostic_session_control_msg` helper alongside the existing `make_tester_present_msg` / `make_communication_control_msg`.
>
> The phase-2 / phase-3 / phase-4 task blocks below reflect the original single-deinit plan; the actual landed code matches the amended design above. See the spec for the authoritative current design.

**Goal:** Restore stock SCC and AEB on HKG CAN-FD HDA II platforms whenever sunnypilot is disengaged, by branching the panda safety code on the existing `controls_allowed` global and emitting UDS edge sequences (`0x10 0x03` + `0x28 0x83 0x01` on engage, `0x28 0x00` + `0x10 0x01` on disengage) on `0x730`.

**Architecture:** Boot-time-set safety flag bit (`HYUNDAI_PARAM_CANFD_DYNAMIC_HANDOFF`) opts the Hyundai CAN-FD safety code into runtime-mutating TX-rejection and forwarding decisions driven by `controls_allowed` (already maintained by the 100 ms heartbeat), and additionally widens the strict 0x730 UDS allowlist to permit four engage/disengage edge patterns. carcontroller cooperates by gating longitudinal-command and tester-present emission on `CC.enabled`, edge-triggering the disengage deinit pair (`0x28 0x00` + `0x10 0x01`), and edge-triggering the engage redisable pair (`0x10 0x03` + `0x28 0x83 0x01`) to replace the boot-time disable that `CarInterface.init` skips under the handoff bit. Per-platform gate is `CP.flags & HyundaiFlags.CANFD_LKA_STEERING` (HDA II), excluding `CANFD_NO_RADAR_DISABLE` and `CANFD_CAMERA_SCC`.

**Tech Stack:** Python (sunnypilot daemons, carcontroller, CarParams, UI), C (opendbc panda safety modes), pytest (Python tests), opendbc `libsafety_py` (panda safety unit tests via a C-shared-object), git submodule structure (`opendbc_repo`, `panda`), sunnypilot fork conventions.

**Five sequentially-mergeable phases.** Each is a no-op for existing users until the next phase activates it. Phase 5 is manual HIL acceptance; not TDD.

**Note on downstream verifiability:** the user chose a single end-to-end plan over per-phase plans. As a result:

1. **Phase 2's "replay against Phase 1 fixtures" test moves into Phase 5 (HIL acceptance)**, because real fixture data does not exist until Phase 1 has been run on a real car. Phase 2 unit tests use synthetic fixtures only.
2. **Carcontroller unit-test scaffolding does not exist in `opendbc/car/hyundai/tests/test_hyundai.py`** (only fingerprint and interface tests do). Tasks 4.3/4.5/4.7 specify behavior tests with `_build_controller` / `_tick` helpers that the implementer must build by adapting the carcontroller construction pattern from openpilot's `process_replay` test infrastructure (e.g. `selfdrive/test/process_replay/process_replay.py`). If building the scaffolding turns out to be disproportionate to the value, fall back to manual verification via a captured drive log and move the behavior assertions into Phase 5's HIL acceptance.
3. **The UI sub-toggle test in Task 4.9 assumes a `HyundaiBrandLayout` constructor that may not match the file's actual shape.** The existing `selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py` should be read first; the test is to be adapted to whatever construction pattern the file uses.

**SCC_CONTROL address is 0x1A0** (verified at `opendbc_repo/opendbc/dbc/generator/hyundai/hyundai_canfd.dbc` line `BO_ 416 SCC_CONTROL: 32 ADRV`). All references to `0x1a0` in this plan are final, not placeholders to verify.

---

## Phase 1 — Trace Capture & Analysis

### Task 1.1: Create the trace-capture package skeleton

**Files:**
- Create: `selfdrive/debug/car/hyundai_canfd_handoff_traces/__init__.py`
- Create: `selfdrive/debug/car/hyundai_canfd_handoff_traces/fixtures/.gitkeep`

- [ ] **Step 1: Create empty package files.**

```bash
mkdir -p selfdrive/debug/car/hyundai_canfd_handoff_traces/fixtures
touch selfdrive/debug/car/hyundai_canfd_handoff_traces/__init__.py
touch selfdrive/debug/car/hyundai_canfd_handoff_traces/fixtures/.gitkeep
```

- [ ] **Step 2: Verify they exist.**

```bash
ls -la selfdrive/debug/car/hyundai_canfd_handoff_traces/
```

Expected: directory containing `__init__.py` and `fixtures/`.

- [ ] **Step 3: Commit.**

```bash
git add selfdrive/debug/car/hyundai_canfd_handoff_traces/__init__.py selfdrive/debug/car/hyundai_canfd_handoff_traces/fixtures/.gitkeep
git commit -m "debug: scaffold HKG CAN-FD HDA II handoff trace package"
```

---

### Task 1.2: Implement `capture.py`

**Files:**
- Create: `selfdrive/debug/car/hyundai_canfd_handoff_traces/capture.py`

- [ ] **Step 1: Write the script.**

```python
#!/usr/bin/env python3
"""Capture raw CAN-FD traffic across SCC engage/disengage cycles on HKG HDA II.

Procedure:
  - openpilot must NOT be running. Stop pandad first.
  - Vehicle ignition on, engine off, key in run position.
  - Run this script with the panda connected over USB.
  - Follow the on-screen prompts to engage/disengage stock SCC.
  - Output: .log file with epoch-timestamped CAN frames suitable for analyze.py.

USE AT YOUR OWN RISK. This script puts the panda in passive listen-only mode
(elm327 safety), so it cannot inject frames -- but the procedure assumes you
are driving the car.
"""
import argparse
import os
import sys
import time
from subprocess import check_output, CalledProcessError

from opendbc.car.structs import CarParams
from panda import Panda


def panda_alive_check():
  try:
    check_output(["pidof", "pandad"])
    print("pandad is running, please kill openpilot before running this script! (aborted)")
    sys.exit(1)
  except CalledProcessError as e:
    if e.returncode != 1:
      raise


def main():
  parser = argparse.ArgumentParser(description="Capture HKG CAN-FD HDA II handoff traces")
  parser.add_argument("--out", type=str, default=None, help="output .log path (default: ./fixtures/capture_<epoch>.log)")
  parser.add_argument("--duration", type=int, default=600, help="capture duration seconds (default: 600)")
  args = parser.parse_args()

  panda_alive_check()

  out_path = args.out or os.path.join(os.path.dirname(__file__), "fixtures", f"capture_{int(time.time())}.log")

  panda = Panda()
  panda.set_safety_mode(CarParams.SafetyModel.elm327)

  print(f"\nCapturing to {out_path}")
  print("Follow procedure: ignition on; stock SCC engage at speed; set/resume cycles; brake-disengage; re-engage; ignition off.")
  print(f"Capture will stop automatically after {args.duration} seconds (or Ctrl+C).\n")

  start = time.monotonic()
  with open(out_path, "w") as f:
    try:
      while (time.monotonic() - start) < args.duration:
        for can in panda.can_recv():
          addr, dat, src = can[0], can[2], can[3]
          f.write(f"{time.time():.6f} {src} {addr:x} {dat.hex()}\n")
    except KeyboardInterrupt:
      print("\nstopped by user")

  print(f"\ndone. captured {os.path.getsize(out_path)} bytes")


if __name__ == "__main__":
  main()
```

- [ ] **Step 2: Verify the script imports and parses args without error.**

```bash
cd /Users/john/Code/sunnypilot
python -c "import selfdrive.debug.car.hyundai_canfd_handoff_traces.capture"
python selfdrive/debug/car/hyundai_canfd_handoff_traces/capture.py --help
```

Expected: imports cleanly; `--help` prints usage.

- [ ] **Step 3: Commit.**

```bash
git add selfdrive/debug/car/hyundai_canfd_handoff_traces/capture.py
git commit -m "debug: add HKG CAN-FD HDA II handoff trace capture script"
```

---

### Task 1.3: Write the failing test for `analyze.py`

**Files:**
- Create: `selfdrive/debug/car/hyundai_canfd_handoff_traces/test_analyze.py`
- Create: `selfdrive/debug/car/hyundai_canfd_handoff_traces/fixtures/synthetic_basic.log` (synthetic fixture for unit tests)

- [ ] **Step 1: Write a tiny synthetic fixture.**

`fixtures/synthetic_basic.log` — three frames spanning an engage transition, in capture format `timestamp bus addr hex`:

```
1700000000.000000 0 730 023e8000000000
1700000001.000000 2 18daf130 0210030000000000
1700000002.000000 2 18daf130 0228030000000000
```

(The first is a tester-present-with-suppress-response to `0x730`. The second is an extended-session-control response. The third is a CommunicationControl disable-rx-tx response. Synthetic — exact bytes are for parser-shape testing only.)

- [ ] **Step 2: Write the failing test.**

```python
#!/usr/bin/env python3
import os
import unittest

from selfdrive.debug.car.hyundai_canfd_handoff_traces.analyze import parse_log, summarize_uds_to_addr


FIXTURE = os.path.join(os.path.dirname(__file__), "fixtures", "synthetic_basic.log")


class TestAnalyze(unittest.TestCase):
  def test_parse_log_returns_three_frames(self):
    frames = parse_log(FIXTURE)
    self.assertEqual(len(frames), 3)
    self.assertEqual(frames[0].addr, 0x730)
    self.assertEqual(frames[0].bus, 0)

  def test_summarize_uds_to_addr_finds_session_and_comm_control(self):
    frames = parse_log(FIXTURE)
    summary = summarize_uds_to_addr(frames, addr=0x730)
    # tester-present to 0x730 is logged
    self.assertGreaterEqual(summary["tester_present_count"], 1)


if __name__ == "__main__":
  unittest.main()
```

- [ ] **Step 3: Run the test, expect failure.**

```bash
cd /Users/john/Code/sunnypilot
python -m pytest selfdrive/debug/car/hyundai_canfd_handoff_traces/test_analyze.py -v
```

Expected: `ModuleNotFoundError: No module named 'selfdrive.debug.car.hyundai_canfd_handoff_traces.analyze'` (or equivalent ImportError).

---

### Task 1.4: Implement `analyze.py` to pass the tests

**Files:**
- Create: `selfdrive/debug/car/hyundai_canfd_handoff_traces/analyze.py`

- [ ] **Step 1: Implement.**

```python
#!/usr/bin/env python3
"""Offline analysis of HKG CAN-FD HDA II handoff traces.

Consumes .log files produced by capture.py and extracts:
  - UDS message sequences to ADAS DRV ECU (0x730) and radar (0x7D0)
  - SCC/FCA/ADRV message presence and counter continuity across engage/disengage transitions
  - The boot-time UDS handshake sunnypilot performs to disable the stock SCC
"""
import argparse
from collections import Counter
from dataclasses import dataclass


@dataclass
class Frame:
  ts: float
  bus: int
  addr: int
  data: bytes


def parse_log(path: str) -> list[Frame]:
  frames: list[Frame] = []
  with open(path) as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      parts = line.split()
      ts = float(parts[0])
      bus = int(parts[1])
      addr = int(parts[2], 16)
      data = bytes.fromhex(parts[3])
      frames.append(Frame(ts=ts, bus=bus, addr=addr, data=data))
  return frames


def summarize_uds_to_addr(frames: list[Frame], addr: int) -> dict:
  """Return counts of UDS service requests targeting a given diagnostic address."""
  counts = Counter()
  for fr in frames:
    if fr.addr != addr:
      continue
    if len(fr.data) < 2:
      continue
    # UDS frames: byte 0 is length, byte 1 is the service ID (with or without 0x80 suppress-response bit).
    service = fr.data[1] & 0x7F
    if service == 0x3E:
      counts["tester_present_count"] += 1
    elif service == 0x10:
      counts["session_control_count"] += 1
    elif service == 0x28:
      counts["comm_control_count"] += 1
    elif service == 0x11:
      counts["ecu_reset_count"] += 1
    else:
      counts[f"service_{service:02x}_count"] += 1
  return dict(counts)


def main():
  parser = argparse.ArgumentParser(description="analyze HKG CAN-FD HDA II handoff trace")
  parser.add_argument("path", type=str, help="path to .log file from capture.py")
  args = parser.parse_args()

  frames = parse_log(args.path)
  print(f"total frames: {len(frames)}")
  print(f"unique addresses: {len({fr.addr for fr in frames})}")
  print(f"UDS to 0x730 (ADAS DRV): {summarize_uds_to_addr(frames, 0x730)}")
  print(f"UDS to 0x7D0 (radar):    {summarize_uds_to_addr(frames, 0x7D0)}")


if __name__ == "__main__":
  main()
```

- [ ] **Step 2: Run the test, expect pass.**

```bash
cd /Users/john/Code/sunnypilot
python -m pytest selfdrive/debug/car/hyundai_canfd_handoff_traces/test_analyze.py -v
```

Expected: 2 tests pass.

- [ ] **Step 3: Commit.**

```bash
git add selfdrive/debug/car/hyundai_canfd_handoff_traces/analyze.py selfdrive/debug/car/hyundai_canfd_handoff_traces/test_analyze.py selfdrive/debug/car/hyundai_canfd_handoff_traces/fixtures/synthetic_basic.log
git commit -m "debug: add HKG CAN-FD HDA II handoff trace analyzer with unit tests"
```

---

### Task 1.5: Document the on-car procedure (`findings.md`)

**Files:**
- Create: `selfdrive/debug/car/hyundai_canfd_handoff_traces/findings.md`

- [ ] **Step 1: Write the procedure-and-template doc.**

```markdown
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
```

- [ ] **Step 2: Commit.**

```bash
git add selfdrive/debug/car/hyundai_canfd_handoff_traces/findings.md
git commit -m "debug: document HKG CAN-FD HDA II handoff trace procedure"
```

---

### Task 1.6: HIL data collection (manual)

- [ ] **Step 1: User executes the stock baseline procedure on a real HDA II vehicle.** Outcome: one `.log` file in `fixtures/` and the corresponding section of `findings.md` filled in.
- [ ] **Step 2: User executes the sunnypilot alpha-long procedure on the same vehicle.** Outcome: one more `.log` file and `findings.md` section.
- [ ] **Step 3: Commit the captured fixtures and findings.**

```bash
git add selfdrive/debug/car/hyundai_canfd_handoff_traces/fixtures/*.log
git add selfdrive/debug/car/hyundai_canfd_handoff_traces/findings.md
git commit -m "debug: capture HKG CAN-FD HDA II handoff trace fixtures from real vehicle"
```

(Phase 2 unit tests do not depend on this, but Phase 5 HIL acceptance does. Mark this task complete only when at least one stock and one sunnypilot capture exist.)

---

## Phase 2 — Hyundai CAN-FD Safety Code

### Task 2.1: Add `HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF`

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/values.py:60-71`

- [ ] **Step 1: Add the new flag.**

Open `opendbc_repo/opendbc/car/hyundai/values.py` and locate the `HyundaiSafetyFlags(IntFlag)` block at lines 60-71. Add a line for the new flag bit:

```python
class HyundaiSafetyFlags(IntFlag):
  EV_GAS = 1
  HYBRID_GAS = 2
  LONG = 4
  CAMERA_SCC = 8
  CANFD_LKA_STEERING = 16
  CANFD_ALT_BUTTONS = 32
  ALT_LIMITS = 64
  CANFD_LKA_STEERING_ALT = 128
  FCEV_GAS = 256
  ALT_LIMITS_2 = 512
  CANFD_DYNAMIC_HANDOFF = 1024
```

- [ ] **Step 2: Verify the import surface still loads.**

```bash
cd /Users/john/Code/sunnypilot
python -c "from opendbc.car.hyundai.values import HyundaiSafetyFlags; print(HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)"
```

Expected: `HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF`

- [ ] **Step 3: Commit.**

```bash
git -C opendbc_repo add opendbc/car/hyundai/values.py
git -C opendbc_repo commit -m "hyundai: add CANFD_DYNAMIC_HANDOFF safety flag bit"
```

---

### Task 2.2: Add the safety flag constant in `hyundai_canfd.h`

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h:228-229`

- [ ] **Step 1: Add the constant.**

In `hyundai_canfd_init()`, alongside the existing flag constants:

```c
static safety_config hyundai_canfd_init(uint16_t param) {
  const uint16_t HYUNDAI_PARAM_CANFD_LKA_STEERING_ALT = 128;
  const uint16_t HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;
  const uint16_t HYUNDAI_PARAM_CANFD_DYNAMIC_HANDOFF = 1024;
```

- [ ] **Step 2: Verify the file still compiles via the libsafety build.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
scons -u opendbc/safety/tests/libsafety
```

Expected: clean build of `libsafety.so`.

- [ ] **Step 3: Commit.**

```bash
git -C opendbc_repo add opendbc/safety/modes/hyundai_canfd.h
git -C opendbc_repo commit -m "hyundai_canfd safety: declare DYNAMIC_HANDOFF param constant"
```

---

### Task 2.3: Add the runtime bool and `GET_FLAG` parse

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h` (file-scope state + init)

- [ ] **Step 1: Add the file-scope bool near the other hyundai_canfd_* statics (above `hyundai_canfd_init`).**

```c
static bool hyundai_canfd_dynamic_handoff = false;
```

- [ ] **Step 2: Add the `GET_FLAG` in `hyundai_canfd_init` alongside the existing parses at lines 277-278.**

```c
hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
hyundai_canfd_lka_steering_alt = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEERING_ALT);
hyundai_canfd_dynamic_handoff = GET_FLAG(param, HYUNDAI_PARAM_CANFD_DYNAMIC_HANDOFF);
```

- [ ] **Step 3: Rebuild libsafety.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
scons -u opendbc/safety/tests/libsafety
```

Expected: clean build.

- [ ] **Step 4: Commit.**

```bash
git -C opendbc_repo add opendbc/safety/modes/hyundai_canfd.h
git -C opendbc_repo commit -m "hyundai_canfd safety: parse DYNAMIC_HANDOFF flag at init"
```

---

### Task 2.4: Write the failing safety unit test for TX gating

**Files:**
- Modify: `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py`

- [ ] **Step 1: Add a new test class at the bottom of the file.**

```python
class TestHyundaiCanfdDynamicHandoff(unittest.TestCase):
  """Coverage for the runtime TX/forwarding gating under CANFD_DYNAMIC_HANDOFF."""

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd")
    self.safety = libsafety_py.libsafety
    # HDA II long config + dynamic handoff bit
    param = (HyundaiSafetyFlags.LONG
             | HyundaiSafetyFlags.CANFD_LKA_STEERING
             | HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, param)

  def _make_scc_control(self):
    return self.packer.make_can_msg_panda("SCC_CONTROL", 1, {"aReqRaw": 0.0})

  def test_scc_control_tx_blocked_when_controls_not_allowed(self):
    self.safety.set_controls_allowed(False)
    self.assertFalse(self.safety.safety_tx_hook(self._make_scc_control()))

  def test_scc_control_tx_allowed_when_controls_allowed(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.safety.safety_tx_hook(self._make_scc_control()))
```

- [ ] **Step 2: Run the test, expect failure.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/safety/tests/test_hyundai_canfd.py::TestHyundaiCanfdDynamicHandoff -v
```

Expected: `test_scc_control_tx_blocked_when_controls_not_allowed` FAILS because the TX hook currently allows SCC_CONTROL whenever it's in the static TX list, regardless of `controls_allowed`.

---

### Task 2.5: Implement TX hook gating on `controls_allowed`

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h:143-225` (`hyundai_canfd_tx_hook`)

- [ ] **Step 1: Locate the SCC_CONTROL handling in `hyundai_canfd_tx_hook`** (around lines 195-205, where longitudinal accel checks happen).

- [ ] **Step 2: Add the `controls_allowed` gate.**

Find the block:

```c
    if (hyundai_longitudinal) {
      violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
```

Add a guard before it:

```c
    if (hyundai_longitudinal) {
      if (hyundai_canfd_dynamic_handoff && !controls_allowed) {
        tx = false;
        return tx;
      }
      violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
```

Apply the same gate to the ADRV control frames (addresses 0x51, 0x160, 0x1EA, 0x200, 0x345, 0x1DA — find the existing TX-hook block that handles them; if no specific block exists, add a default-deny at the end of the hook that returns false when `hyundai_canfd_dynamic_handoff && !controls_allowed && addr in {0x51, 0x160, 0x1EA, 0x200, 0x345, 0x1DA}`).

- [ ] **Step 3: Rebuild libsafety.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
scons -u opendbc/safety/tests/libsafety
```

- [ ] **Step 4: Run the tests, expect pass.**

```bash
python -m pytest opendbc/safety/tests/test_hyundai_canfd.py::TestHyundaiCanfdDynamicHandoff -v
```

Expected: both tests pass.

- [ ] **Step 5: Commit.**

```bash
git -C opendbc_repo add opendbc/safety/modes/hyundai_canfd.h opendbc/safety/tests/test_hyundai_canfd.py
git -C opendbc_repo commit -m "hyundai_canfd safety: gate longitudinal TX on controls_allowed under dynamic handoff"
```

---

### Task 2.6: Write the failing test for forwarding behavior

**Files:**
- Modify: `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py`

- [ ] **Step 1: Add forwarding tests to `TestHyundaiCanfdDynamicHandoff`.**

```python
  def test_stock_scc_forwarded_when_disengaged(self):
    # SCC_CONTROL from bus 2 (camera/radar side) should pass through when controls_allowed=False
    self.safety.set_controls_allowed(False)
    # safety_fwd_hook returns -1 for blocked, else destination bus
    result = self.safety.safety_fwd_hook(2, 0x1a0)  # SCC_CONTROL
    self.assertNotEqual(result, -1, "stock SCC should forward when openpilot is disengaged")

  def test_stock_scc_blocked_when_engaged(self):
    self.safety.set_controls_allowed(True)
    result = self.safety.safety_fwd_hook(2, 0x1a0)
    self.assertEqual(result, -1, "stock SCC should be blocked when openpilot is engaged")
```

(Verify `0x1a0` is the actual SCC_CONTROL CAN-FD address by checking `_hyundai_canfd_common.dbc`. If different, substitute.)

- [ ] **Step 2: Run, expect failure.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/safety/tests/test_hyundai_canfd.py::TestHyundaiCanfdDynamicHandoff -v -k forward
```

Expected: `test_stock_scc_forwarded_when_disengaged` FAILS because the default `safety_fwd_hook` blocks the address (it's in the TX allow-list with `check_relay=true`).

---

### Task 2.7: Mark longitudinal TX entries with `disable_static_blocking=true`

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h:239-250` (`HYUNDAI_CANFD_LKA_STEERING_LONG_TX_MSGS`)

- [ ] **Step 1: Locate the long TX list.**

Find:

```c
static const CanMsg HYUNDAI_CANFD_LKA_STEERING_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEERING_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(1)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(1, true)
    {0x51,  0, 32, .check_relay = false},  // ADRV_0x51
    {0x730, 1,  8, .check_relay = false},  // tester present for ADAS ECU disable
    {0x160, 1, 16, .check_relay = false},  // ADRV_0x160
    {0x1EA, 1, 32, .check_relay = false},  // ADRV_0x1ea
    {0x200, 1,  8, .check_relay = false},  // ADRV_0x200
    {0x345, 1,  8, .check_relay = false},  // ADRV_0x345
    {0x1DA, 1, 32, .check_relay = false},  // ADRV_0x1da
  };
```

- [ ] **Step 2: Add `disable_static_blocking = true` to the ADRV entries.**

```c
    {0x51,  0, 32, .check_relay = false, .disable_static_blocking = true},
    {0x730, 1,  8, .check_relay = false},  // UDS path; always allow TX
    {0x160, 1, 16, .check_relay = false, .disable_static_blocking = true},
    {0x1EA, 1, 32, .check_relay = false, .disable_static_blocking = true},
    {0x200, 1,  8, .check_relay = false, .disable_static_blocking = true},
    {0x345, 1,  8, .check_relay = false, .disable_static_blocking = true},
    {0x1DA, 1, 32, .check_relay = false, .disable_static_blocking = true},
```

For the SCC_CONTROL macro entry (`HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS`), check the macro definition near the top of the file (lines 23-26). If the macro produces entries with `check_relay=true`, update the macro to take a `disable_static_blocking` parameter or add a parallel macro. Simplest: define a new macro `HYUNDAI_CANFD_SCC_CONTROL_HANDOFF_TX_MSGS(bus, longitudinal)` that mirrors the existing one but sets `disable_static_blocking=true`, and use it in `HYUNDAI_CANFD_LKA_STEERING_LONG_TX_MSGS`.

- [ ] **Step 3: Rebuild libsafety.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
scons -u opendbc/safety/tests/libsafety
```

- [ ] **Step 4: Commit.**

```bash
git -C opendbc_repo add opendbc/safety/modes/hyundai_canfd.h
git -C opendbc_repo commit -m "hyundai_canfd safety: mark HDA II long TX entries disable_static_blocking for handoff"
```

---

### Task 2.8: Add the `fwd` hook to `hyundai_canfd_hooks`

**Files:**
- Modify: `opendbc_repo/opendbc/safety/modes/hyundai_canfd.h` (new function + hooks struct update)

- [ ] **Step 1: Implement the fwd hook.**

Add above `hyundai_canfd_init`:

```c
static bool hyundai_canfd_fwd_hook(int bus_num, int addr) {
  bool block = false;

  if (hyundai_canfd_dynamic_handoff) {
    // Longitudinal addresses we own when engaged: SCC_CONTROL and the ADRV cluster.
    const int handoff_addrs[] = {0x1a0, 0x51, 0x160, 0x1EA, 0x200, 0x345, 0x1DA};
    for (size_t i = 0; i < sizeof(handoff_addrs) / sizeof(handoff_addrs[0]); i++) {
      if (addr == handoff_addrs[i]) {
        block = controls_allowed;
        break;
      }
    }
  }

  (void)bus_num;  // not used; framework already routes via get_fwd_bus
  return block;
}
```

(Replace `0x1a0` with the real SCC_CONTROL address verified in Task 2.6 if different.)

- [ ] **Step 2: Wire the hook into `hyundai_canfd_hooks` (lines 384-391).**

```c
const safety_hooks hyundai_canfd_hooks = {
  .init = hyundai_canfd_init,
  .rx = hyundai_canfd_rx_hook,
  .tx = hyundai_canfd_tx_hook,
  .fwd = hyundai_canfd_fwd_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
```

- [ ] **Step 3: Rebuild and run the forwarding tests.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
scons -u opendbc/safety/tests/libsafety
python -m pytest opendbc/safety/tests/test_hyundai_canfd.py::TestHyundaiCanfdDynamicHandoff -v
```

Expected: all four tests in the class pass.

- [ ] **Step 4: Commit.**

```bash
git -C opendbc_repo add opendbc/safety/modes/hyundai_canfd.h opendbc/safety/tests/test_hyundai_canfd.py
git -C opendbc_repo commit -m "hyundai_canfd safety: add fwd hook for dynamic radar handoff"
```

---

### Task 2.9: Regression coverage — flag-unset behavior unchanged

**Files:**
- Modify: `opendbc_repo/opendbc/safety/tests/test_hyundai_canfd.py`

- [ ] **Step 1: Add a regression test class with the flag unset.**

```python
class TestHyundaiCanfdNoHandoffRegression(unittest.TestCase):
  """When CANFD_DYNAMIC_HANDOFF is unset, behavior must match the historical alpha-long path."""

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd")
    self.safety = libsafety_py.libsafety
    param = HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_LKA_STEERING
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, param)

  def test_scc_control_tx_allowed_regardless_of_controls_allowed(self):
    msg = self.packer.make_can_msg_panda("SCC_CONTROL", 1, {"aReqRaw": 0.0})
    self.safety.set_controls_allowed(False)
    self.assertTrue(self.safety.safety_tx_hook(msg))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.safety.safety_tx_hook(msg))

  def test_stock_scc_blocked_regardless_of_controls_allowed(self):
    self.safety.set_controls_allowed(False)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x1a0), -1)
    self.safety.set_controls_allowed(True)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x1a0), -1)
```

- [ ] **Step 2: Run.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/safety/tests/test_hyundai_canfd.py::TestHyundaiCanfdNoHandoffRegression -v
```

Expected: both tests pass. If they don't, the new code regressed the alpha-long-without-handoff path — fix before continuing.

- [ ] **Step 3: Commit.**

```bash
git -C opendbc_repo add opendbc/safety/tests/test_hyundai_canfd.py
git -C opendbc_repo commit -m "hyundai_canfd safety: regression coverage for handoff-disabled alpha-long users"
```

---

## Phase 3 — CarParams Wiring

### Task 3.1: Write the failing CarParams test

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py`

- [ ] **Step 1: Add a test that asserts the flag bit conditions.**

```python
class TestHyundaiCarParamsDynamicHandoff(unittest.TestCase):
  """CarParams safety_param should include CANFD_DYNAMIC_HANDOFF only when all five conditions are met."""

  def _build(self, has_hda2, no_radar_disable, camera_scc, handoff_param, alpha_long_param):
    from opendbc.car.hyundai.interface import CarInterface
    from opendbc.car.hyundai.values import CAR, HyundaiFlags
    from openpilot.common.params import Params

    # Use GV70 Electrified as the platform under test
    fingerprint = {0: {}, 1: {}, 2: {}}
    params_obj = Params()
    params_obj.put_bool("DynamicRadarHandoffEnabled", handoff_param)
    params_obj.put_bool("AlphaLongitudinalEnabled", alpha_long_param)
    cp = CarInterface.get_params(CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN, fingerprint, [], False)

    if has_hda2:
      cp.flags |= HyundaiFlags.CANFD_LKA_STEERING.value
    if no_radar_disable:
      cp.flags |= HyundaiFlags.CANFD_NO_RADAR_DISABLE.value
    if camera_scc:
      cp.flags |= HyundaiFlags.CANFD_CAMERA_SCC.value

    # Re-derive safety_param from the (mutated) CP
    cp = CarInterface.apply_safety_param_overlays(cp, params_obj)
    return cp

  def test_all_conditions_met_sets_bit(self):
    cp = self._build(has_hda2=True, no_radar_disable=False, camera_scc=False,
                     handoff_param=True, alpha_long_param=True)
    self.assertTrue(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)

  def test_handoff_param_off_clears_bit(self):
    cp = self._build(has_hda2=True, no_radar_disable=False, camera_scc=False,
                     handoff_param=False, alpha_long_param=True)
    self.assertFalse(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)

  def test_alpha_long_off_clears_bit(self):
    cp = self._build(has_hda2=True, no_radar_disable=False, camera_scc=False,
                     handoff_param=True, alpha_long_param=False)
    self.assertFalse(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)

  def test_no_hda2_clears_bit(self):
    cp = self._build(has_hda2=False, no_radar_disable=False, camera_scc=False,
                     handoff_param=True, alpha_long_param=True)
    self.assertFalse(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)

  def test_no_radar_disable_clears_bit(self):
    cp = self._build(has_hda2=True, no_radar_disable=True, camera_scc=False,
                     handoff_param=True, alpha_long_param=True)
    self.assertFalse(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)

  def test_camera_scc_clears_bit(self):
    cp = self._build(has_hda2=True, no_radar_disable=False, camera_scc=True,
                     handoff_param=True, alpha_long_param=True)
    self.assertFalse(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
```

(The exact CarParams construction approach may differ in the actual codebase — adapt to use the same pattern existing tests use to build a CarParams instance.)

- [ ] **Step 2: Run, expect failure.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_hyundai.py::TestHyundaiCarParamsDynamicHandoff -v
```

Expected: `AttributeError` or `test_all_conditions_met_sets_bit` FAILS because `apply_safety_param_overlays` (or equivalent) does not yet set the bit.

---

### Task 3.2: Implement the CarParams conditional

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/interface.py:81-84`

- [ ] **Step 1: Locate the existing safety_param construction.**

Lines 81-84 read:

```python
      if ret.flags & HyundaiFlags.CANFD_LKA_STEERING:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_LKA_STEERING.value
        if ret.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT:
          ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT.value
```

- [ ] **Step 2: Add the dynamic-handoff gate after that block.**

```python
      if ret.flags & HyundaiFlags.CANFD_LKA_STEERING:
        ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_LKA_STEERING.value
        if ret.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT:
          ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT.value

        # Dynamic Radar Handoff opt-in: HDA II only, no radar-disable-blocked platform, not camera SCC.
        if (not (ret.flags & HyundaiFlags.CANFD_NO_RADAR_DISABLE)
            and not (ret.flags & HyundaiFlags.CANFD_CAMERA_SCC)
            and params.get_bool("DynamicRadarHandoffEnabled")
            and params.get_bool("AlphaLongitudinalEnabled")):
          ret.safetyConfigs[-1].safetyParam |= HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF.value
```

(Verify `params` is in scope at this point; if not, accept it as a parameter to the method or use the existing pattern in the file for param access.)

- [ ] **Step 3: Run the tests, expect pass.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_hyundai.py::TestHyundaiCarParamsDynamicHandoff -v
```

Expected: all 6 tests pass.

- [ ] **Step 4: Commit.**

```bash
git -C opendbc_repo add opendbc/car/hyundai/interface.py opendbc/car/hyundai/tests/test_hyundai.py
git -C opendbc_repo commit -m "hyundai: set CANFD_DYNAMIC_HANDOFF safety bit when conditions are met"
```

---

## Phase 4 — carcontroller + UI

### Task 4.1: Write the failing test for `make_communication_control_msg`

**Files:**
- Modify: `opendbc_repo/opendbc/car/tests/test_init.py` (or create if absent — locate the test file for `opendbc/car/__init__.py`)

- [ ] **Step 1: Find or create the test file.**

```bash
find /Users/john/Code/sunnypilot/opendbc_repo -name "test_init.py" -path "*/opendbc/car/*"
```

If none exists, create `opendbc_repo/opendbc/car/tests/test_helpers.py`.

- [ ] **Step 2: Write the failing test.**

```python
import unittest
from opendbc.car import make_communication_control_msg, CanData


class TestCommunicationControlMsg(unittest.TestCase):
  def test_enable_rx_tx_with_suppress_response(self):
    msg = make_communication_control_msg(0x730, 0, sub_function=0x00, suppress_response=True)
    self.assertIsInstance(msg, CanData)
    self.assertEqual(msg.address, 0x730)
    self.assertEqual(msg.src, 0)
    # ISO 14229-1: byte 0 = length (3), byte 1 = 0x28 (CommunicationControl), byte 2 = subFunction | 0x80 (suppress),
    # byte 3 = communicationType (0x01 = normal communication messages, all subnets)
    self.assertEqual(msg.dat[0], 0x03)
    self.assertEqual(msg.dat[1], 0x28)
    self.assertEqual(msg.dat[2], 0x80)  # subFunction 0x00 | suppress 0x80
    self.assertEqual(msg.dat[3], 0x01)

  def test_disable_rx_tx_no_suppress(self):
    msg = make_communication_control_msg(0x730, 0, sub_function=0x03, suppress_response=False)
    self.assertEqual(msg.dat[2], 0x03)
```

- [ ] **Step 3: Run, expect failure.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/tests/test_helpers.py -v
```

Expected: `ImportError: cannot import name 'make_communication_control_msg' from 'opendbc.car'`.

---

### Task 4.2: Implement `make_communication_control_msg`

**Files:**
- Modify: `opendbc_repo/opendbc/car/__init__.py:99-106` (insert below `make_tester_present_msg`)

- [ ] **Step 1: Add the function.**

```python
def make_communication_control_msg(addr, bus, sub_function, communication_type=0x01, suppress_response=False):
  """UDS service 0x28 CommunicationControl.

  sub_function:
    0x00 = enableRxAndTx
    0x01 = enableRxAndDisableTx
    0x02 = disableRxAndEnableTx
    0x03 = disableRxAndTx

  communication_type: per ISO 14229-1. 0x01 = normal communication messages, all subnets.
  """
  sf = sub_function | (0x80 if suppress_response else 0x00)
  dat = [0x03, 0x28, sf, communication_type]
  dat.extend([0x0] * (8 - len(dat)))
  return CanData(addr, bytes(dat), bus)
```

- [ ] **Step 2: Run the test, expect pass.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/tests/test_helpers.py -v
```

Expected: both tests pass.

- [ ] **Step 3: Commit.**

```bash
git -C opendbc_repo add opendbc/car/__init__.py opendbc/car/tests/test_helpers.py
git -C opendbc_repo commit -m "opendbc: add make_communication_control_msg UDS helper"
```

---

### Task 4.3: Write failing carcontroller test — gate longitudinal commands on engagement

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py`

- [ ] **Step 1: Add a test class for carcontroller dynamic-handoff behavior.**

```python
class TestHyundaiCarControllerDynamicHandoff(unittest.TestCase):
  """Verify carcontroller honors the DynamicRadarHandoffEnabled param + CC.enabled."""

  def _build_controller(self, dynamic_handoff: bool):
    # Construct a CarController with HDA II flags, alpha long enabled, and dynamic handoff param set
    # (mirror the pattern existing tests use in this file)
    ...

  def _tick(self, controller, enabled: bool, frame: int):
    # Drive one update() with the given enabled state and frame count; return list of CAN frames emitted
    ...

  def test_no_scc_control_when_disengaged_under_handoff(self):
    cc = self._build_controller(dynamic_handoff=True)
    sends = self._tick(cc, enabled=False, frame=100)
    addrs = {s.address for s in sends}
    self.assertNotIn(0x1a0, addrs)  # SCC_CONTROL

  def test_scc_control_emitted_when_engaged_under_handoff(self):
    cc = self._build_controller(dynamic_handoff=True)
    sends = self._tick(cc, enabled=True, frame=100)
    addrs = {s.address for s in sends}
    self.assertIn(0x1a0, addrs)

  def test_scc_control_emitted_when_disengaged_without_handoff(self):
    # Today's behavior preserved when the param is off
    cc = self._build_controller(dynamic_handoff=False)
    sends = self._tick(cc, enabled=False, frame=100)
    addrs = {s.address for s in sends}
    self.assertIn(0x1a0, addrs)
```

(Fill in `_build_controller` and `_tick` by mirroring existing test patterns in the file — these tests exist for other parts of carcontroller, so use the same scaffolding.)

- [ ] **Step 2: Run, expect failure.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_hyundai.py::TestHyundaiCarControllerDynamicHandoff -v
```

Expected: `test_no_scc_control_when_disengaged_under_handoff` FAILS — today's carcontroller emits SCC_CONTROL regardless of engaged state when `openpilotLongitudinalControl` is on.

---

### Task 4.4: Implement longitudinal-command gating in carcontroller

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/carcontroller.py` (in `create_canfd_msgs` / the longitudinal-frame synthesis path)

- [ ] **Step 1: Locate `create_canfd_msgs` (called from `carcontroller.py:122`).** Find where SCC_CONTROL and ADRV frames are appended.

- [ ] **Step 2: Add a gate.**

```python
# Skip longitudinal frame synthesis when the dynamic-handoff param is on and sunnypilot is disengaged.
if self.dynamic_radar_handoff_enabled and not CC.enabled:
  pass  # leave can_sends untouched for longitudinal addresses
else:
  # existing SCC_CONTROL / ADRV emission
  ...
```

`self.dynamic_radar_handoff_enabled` is read once at controller init from `Params().get_bool("DynamicRadarHandoffEnabled")` and stored — matches the pattern other carcontroller params use.

- [ ] **Step 3: Run the tests, expect pass.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_hyundai.py::TestHyundaiCarControllerDynamicHandoff -v
```

Expected: all three tests pass.

- [ ] **Step 4: Commit.**

```bash
git -C opendbc_repo add opendbc/car/hyundai/carcontroller.py opendbc/car/hyundai/tests/test_hyundai.py
git -C opendbc_repo commit -m "hyundai carcontroller: gate longitudinal commands on engagement under dynamic handoff"
```

---

### Task 4.5: Write failing test — gate tester-present on engagement

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py`

- [ ] **Step 1: Add to `TestHyundaiCarControllerDynamicHandoff`.**

```python
  def test_no_tester_present_when_disengaged_under_handoff(self):
    cc = self._build_controller(dynamic_handoff=True)
    # frame=100 is the 1Hz tester-present tick
    sends = self._tick(cc, enabled=False, frame=100)
    addrs = {s.address for s in sends}
    self.assertNotIn(0x730, addrs)

  def test_tester_present_when_engaged_under_handoff(self):
    cc = self._build_controller(dynamic_handoff=True)
    sends = self._tick(cc, enabled=True, frame=100)
    addrs = {s.address for s in sends}
    self.assertIn(0x730, addrs)

  def test_tester_present_when_disengaged_without_handoff(self):
    # Today's behavior preserved when the param is off (tester-present always emitted on long path)
    cc = self._build_controller(dynamic_handoff=False)
    sends = self._tick(cc, enabled=False, frame=100)
    addrs = {s.address for s in sends}
    self.assertIn(0x730, addrs)
```

- [ ] **Step 2: Run, expect failure.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_hyundai.py::TestHyundaiCarControllerDynamicHandoff -v -k tester
```

Expected: `test_no_tester_present_when_disengaged_under_handoff` FAILS — today's carcontroller emits tester-present unconditionally while `openpilotLongitudinalControl` is on.

---

### Task 4.6: Implement tester-present gating

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/carcontroller.py:107-114`

- [ ] **Step 1: Update the gate.**

```python
    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not ((self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) or self.ESCC.enabled) and \
            self.CP.openpilotLongitudinalControl and \
            (not self.dynamic_radar_handoff_enabled or CC.enabled):
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))
```

- [ ] **Step 2: Run, expect pass.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_hyundai.py::TestHyundaiCarControllerDynamicHandoff -v
```

Expected: all 6 tests so far pass.

- [ ] **Step 3: Commit.**

```bash
git -C opendbc_repo add opendbc/car/hyundai/carcontroller.py opendbc/car/hyundai/tests/test_hyundai.py
git -C opendbc_repo commit -m "hyundai carcontroller: gate tester-present on engagement under dynamic handoff"
```

---

### Task 4.7: Write failing test — UDS deinit on disengage edge

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/tests/test_hyundai.py`

- [ ] **Step 1: Add to `TestHyundaiCarControllerDynamicHandoff`.**

```python
  def _is_uds_28_00(self, can_data):
    # UDS service 0x28 (CommunicationControl), subFunction 0x00 (enableRxAndTx), suppress-response bit = 0x80
    return (can_data.address == 0x730
            and len(can_data.dat) >= 4
            and can_data.dat[0] == 0x03
            and can_data.dat[1] == 0x28
            and (can_data.dat[2] & 0x7F) == 0x00)

  def test_uds_deinit_emitted_on_disengage_edge(self):
    cc = self._build_controller(dynamic_handoff=True)
    self._tick(cc, enabled=True, frame=100)   # engaged
    sends = self._tick(cc, enabled=False, frame=101)  # disengage transition
    deinit_frames = [s for s in sends if self._is_uds_28_00(s)]
    self.assertEqual(len(deinit_frames), 1, "exactly one UDS 0x28 0x00 frame on engage→disengage edge")

  def test_uds_deinit_not_repeated_while_disengaged(self):
    cc = self._build_controller(dynamic_handoff=True)
    self._tick(cc, enabled=True, frame=100)
    self._tick(cc, enabled=False, frame=101)  # disengage edge
    sends = self._tick(cc, enabled=False, frame=102)  # next tick, still disengaged
    deinit_frames = [s for s in sends if self._is_uds_28_00(s)]
    self.assertEqual(len(deinit_frames), 0, "deinit must not repeat after the edge")

  def test_uds_deinit_not_emitted_without_handoff_param(self):
    cc = self._build_controller(dynamic_handoff=False)
    self._tick(cc, enabled=True, frame=100)
    sends = self._tick(cc, enabled=False, frame=101)
    deinit_frames = [s for s in sends if self._is_uds_28_00(s)]
    self.assertEqual(len(deinit_frames), 0)
```

- [ ] **Step 2: Run, expect failure.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_hyundai.py::TestHyundaiCarControllerDynamicHandoff -v -k deinit
```

Expected: `test_uds_deinit_emitted_on_disengage_edge` FAILS — carcontroller emits no UDS 0x28 frame today.

---

### Task 4.8: Implement UDS deinit on disengage edge

**Files:**
- Modify: `opendbc_repo/opendbc/car/hyundai/carcontroller.py`

- [ ] **Step 1: Add `prev_enabled` state to the controller and edge-detect.**

In `__init__`:

```python
self.dynamic_radar_handoff_enabled = Params().get_bool("DynamicRadarHandoffEnabled")
self.prev_enabled = False
```

In `update()` (early in the function, before `can_sends` is built):

```python
disengage_edge = (self.prev_enabled and not CC.enabled
                  and self.dynamic_radar_handoff_enabled
                  and self.CP.openpilotLongitudinalControl
                  and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC)
                  and (self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING))
self.prev_enabled = CC.enabled
```

Then after the existing tester-present block, append:

```python
if disengage_edge:
  # Re-enable normal communication on the ADAS DRV ECU after sunnypilot has been driving.
  can_sends.append(make_communication_control_msg(0x730, self.CAN.ECAN,
                                                  sub_function=0x00,
                                                  suppress_response=True))
```

- [ ] **Step 2: Add the import.**

At the top of `carcontroller.py`, alongside `make_tester_present_msg`:

```python
from opendbc.car import Bus, DT_CTRL, make_communication_control_msg, make_tester_present_msg, structs
```

- [ ] **Step 3: Run, expect pass.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/car/hyundai/tests/test_hyundai.py::TestHyundaiCarControllerDynamicHandoff -v
```

Expected: all 9 tests in the class pass.

- [ ] **Step 4: Commit.**

```bash
git -C opendbc_repo add opendbc/car/hyundai/carcontroller.py opendbc/car/hyundai/tests/test_hyundai.py
git -C opendbc_repo commit -m "hyundai carcontroller: emit UDS CommunicationControl deinit on disengage"
```

---

### Task 4.9: Write the failing UI visibility test

**Files:**
- Create: `selfdrive/ui/tests/test_hyundai_brand_layout.py`

- [ ] **Step 1: Write the test.**

```python
#!/usr/bin/env python3
import unittest
from unittest.mock import MagicMock

from selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.hyundai import HyundaiBrandLayout
from opendbc.car.hyundai.values import HyundaiFlags


class TestHyundaiBrandLayoutDynamicHandoffToggle(unittest.TestCase):
  def _layout(self, flags, alpha_long_enabled):
    cp = MagicMock()
    cp.flags = flags
    cp.alphaLongitudinalAvailable = True
    layout = HyundaiBrandLayout(cp=cp, alpha_long_enabled=alpha_long_enabled)
    return layout

  def test_visible_when_hda2_and_alpha_long_on(self):
    layout = self._layout(flags=HyundaiFlags.CANFD_LKA_STEERING.value, alpha_long_enabled=True)
    self.assertTrue(layout.dynamic_handoff_toggle.visible)

  def test_hidden_when_not_hda2(self):
    layout = self._layout(flags=0, alpha_long_enabled=True)
    self.assertFalse(layout.dynamic_handoff_toggle.visible)

  def test_hidden_when_alpha_long_off(self):
    layout = self._layout(flags=HyundaiFlags.CANFD_LKA_STEERING.value, alpha_long_enabled=False)
    self.assertFalse(layout.dynamic_handoff_toggle.visible)

  def test_hidden_when_no_radar_disable_flag(self):
    flags = HyundaiFlags.CANFD_LKA_STEERING.value | HyundaiFlags.CANFD_NO_RADAR_DISABLE.value
    layout = self._layout(flags=flags, alpha_long_enabled=True)
    self.assertFalse(layout.dynamic_handoff_toggle.visible)

  def test_hidden_when_camera_scc(self):
    flags = HyundaiFlags.CANFD_LKA_STEERING.value | HyundaiFlags.CANFD_CAMERA_SCC.value
    layout = self._layout(flags=flags, alpha_long_enabled=True)
    self.assertFalse(layout.dynamic_handoff_toggle.visible)


if __name__ == "__main__":
  unittest.main()
```

(If the actual `HyundaiBrandLayout` class doesn't expose a constructor like this, adapt to whatever pattern the existing UI uses. The test's intent is: assert the visibility under each combination of inputs.)

- [ ] **Step 2: Run, expect failure (import error since the toggle doesn't exist yet).**

```bash
cd /Users/john/Code/sunnypilot
python -m pytest selfdrive/ui/tests/test_hyundai_brand_layout.py -v
```

Expected: ImportError or AttributeError on `dynamic_handoff_toggle`.

---

### Task 4.10: Implement the UI sub-toggle

**Files:**
- Modify: `selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py`

- [ ] **Step 1: Read the existing file to understand the pattern.**

```bash
cat /Users/john/Code/sunnypilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py
```

- [ ] **Step 2: Add the sub-toggle following the existing tuning-mode toggle pattern at lines 34-59.**

```python
self.dynamic_handoff_toggle = ParamControlSP(
    "DynamicRadarHandoffEnabled",
    "Dynamic Radar Handoff",
    "Restores stock SCC and AEB when sunnypilot is disengaged. AEB may not re-arm reliably "
    "after disengagement; stock behavior is hardware-dependent. Out of scope: stock AEB is not "
    "preserved while sunnypilot is engaged.",
    icon="icons/radar.png",  # or whatever icon convention this file uses
)

# Visibility computation:
self.dynamic_handoff_toggle.set_visible(
    alpha_long_enabled
    and bool(cp.flags & HyundaiFlags.CANFD_LKA_STEERING)
    and not bool(cp.flags & HyundaiFlags.CANFD_NO_RADAR_DISABLE)
    and not bool(cp.flags & HyundaiFlags.CANFD_CAMERA_SCC)
)
```

- [ ] **Step 3: Run, expect pass.**

```bash
cd /Users/john/Code/sunnypilot
python -m pytest selfdrive/ui/tests/test_hyundai_brand_layout.py -v
```

Expected: all 5 tests pass.

- [ ] **Step 4: Commit.**

```bash
git add selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py selfdrive/ui/tests/test_hyundai_brand_layout.py
git commit -m "ui hyundai: add Dynamic Radar Handoff sub-toggle for HDA II platforms"
```

---

### Task 4.11: End-to-end smoke check

- [ ] **Step 1: Run the full opendbc test suite to catch regressions.**

```bash
cd /Users/john/Code/sunnypilot/opendbc_repo
python -m pytest opendbc/safety/tests/test_hyundai_canfd.py opendbc/car/hyundai/tests/test_hyundai.py opendbc/car/tests/test_helpers.py -v
```

Expected: all pass.

- [ ] **Step 2: Run the sunnypilot UI tests.**

```bash
cd /Users/john/Code/sunnypilot
python -m pytest selfdrive/ui/tests/test_hyundai_brand_layout.py selfdrive/debug/car/hyundai_canfd_handoff_traces/test_analyze.py -v
```

Expected: all pass.

- [ ] **Step 3: No commit.** This is a verification gate before moving to HIL.

---

## Phase 5 — Hardware-in-the-Loop Acceptance

(Not TDD. Manual procedure on a real HDA II HKG CAN-FD vehicle, scan tool required. See spec section "Phase 5 — Hardware-in-the-loop acceptance" for full criteria.)

### Task 5.1: Baseline drive

- [ ] **Step 1:** With `DynamicRadarHandoffEnabled=false` and `AlphaLongitudinalEnabled=false`, drive ≥15 min with ≥5 stock SCC engage/disengage cycles.
- [ ] **Step 2:** Scan tool readout of the full module list at end of drive. Record any new SCC, FCA, or ADAS DRV DTCs.
- [ ] **Step 3:** Pass criterion: zero new DTCs. Record in `findings.md`.

### Task 5.2: Three test drives

- [ ] **Step 1:** Enable `DynamicRadarHandoffEnabled` and `AlphaLongitudinalEnabled`. Drive ≥15 min on three consecutive ignition cycles, with ≥5 sunnypilot engage/disengage cycles per drive.
- [ ] **Step 2:** Scan tool readout after each drive. Record DTCs.
- [ ] **Step 3:** During each test drive, after at least one disengage following an engage, visually confirm the stock collision-warning indicator on the cluster is in its normal armed state.
- [ ] **Step 4:** Pass criterion for each drive: zero new SCC, FCA, or ADAS DRV DTCs; cluster indicator normal.

### Task 5.3: Promote or hold

- [ ] **Step 1: If all four drives pass (baseline + three test):** move the "Dynamic Radar Handoff" sub-toggle out of the developer panel into the standard Vehicle → Hyundai settings (edit `selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/hyundai.py` to remove the developer-panel gating, if applicable). Commit.
- [ ] **Step 2: If any drive fails:** record the failure mode in `findings.md`. Common follow-ups:
  - Prepend `UDS 0x10 0x01` (defaultSession) before the `0x28 0x00` deinit — one-line carcontroller change.
  - Investigate alternate sub-function values for `0x28` (some ECUs require different `communicationType` values).
  - Capture additional traces and re-analyze.
  Sub-toggle stays in developer panel; the failure mode becomes a follow-up spec or a new Phase 1 capture pass.

---

## End of Plan

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

  p = Panda()
  p.set_safety_mode(CarParams.SafetyModel.elm327)

  print(f"\nCapturing to {out_path}")
  print("Follow procedure: ignition on; stock SCC engage at speed; set/resume cycles; brake-disengage; re-engage; ignition off.")
  print(f"Capture will stop automatically after {args.duration} seconds (or Ctrl+C).\n")

  start = time.monotonic()
  with open(out_path, "w") as f:
    try:
      while (time.monotonic() - start) < args.duration:
        for can in p.can_recv():
          addr, dat, src = can[0], can[2], can[3]
          f.write(f"{time.time():.6f} {src} {addr:x} {dat.hex()}\n")
    except KeyboardInterrupt:
      print("\nstopped by user")

  print(f"\ndone. captured {os.path.getsize(out_path)} bytes")


if __name__ == "__main__":
  main()

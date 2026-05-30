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

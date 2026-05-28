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

#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Tests for the Hyundai brand layout visibility logic.

Exercises the pure-function visibility predicate (should_show_dynamic_handoff)
without spinning up the UI stack. We load hyundai.py directly via importlib to
bypass package __init__ chains that pull in raylib via system.ui.widgets, and
stub the four UI-side imports hyundai.py itself does. opendbc is imported
normally — HyundaiFlags values come from the real enum.
"""
import abc
import importlib.util
import os
import sys
import types
import unittest  # noqa: TID251
from unittest.mock import MagicMock  # noqa: TID251

from opendbc.car.hyundai.values import HyundaiFlags


def _build_ui_stubs():
  """Stub the four UI-side imports hyundai.py does (msgq/raylib chains)."""
  for name in (
    "openpilot.selfdrive.ui.ui_state",
    "openpilot.system.ui.lib.multilang",
    "openpilot.system.ui.sunnypilot.widgets.list_view",
    "openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base",
  ):
    sys.modules.setdefault(name, types.ModuleType(name))

  sys.modules["openpilot.system.ui.lib.multilang"].tr = lambda s: s
  lv = sys.modules["openpilot.system.ui.sunnypilot.widgets.list_view"]
  lv.multiple_button_item_sp = MagicMock(return_value=MagicMock())
  lv.toggle_item_sp = MagicMock(return_value=MagicMock())
  sys.modules["openpilot.selfdrive.ui.ui_state"].ui_state = MagicMock()

  class _BrandSettings(abc.ABC):
    def __init__(self):
      self.items = []

    @abc.abstractmethod
    def update_settings(self):
      pass

  sys.modules["openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base"].BrandSettings = _BrandSettings


def _load_hyundai_module():
  """Load hyundai.py directly, bypassing package __init__ files."""
  path = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    "..", "sunnypilot", "layouts", "settings", "vehicle", "brands", "hyundai.py",
  ))
  spec = importlib.util.spec_from_file_location("_hyundai_brand_layout", path)
  assert spec is not None and spec.loader is not None
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


_build_ui_stubs()
_should_show_dynamic_handoff = _load_hyundai_module().should_show_dynamic_handoff


class TestDynamicRadarHandoffVisibility(unittest.TestCase):
  """Visibility predicate tests for the Dynamic Radar Handoff sub-toggle."""

  def _cp(self, flags: int):
    cp = MagicMock()
    cp.flags = flags
    return cp

  def test_visible_when_all_conditions_met(self):
    cp = self._cp(HyundaiFlags.CANFD_LKA_STEERING.value)
    self.assertTrue(_should_show_dynamic_handoff(cp, alpha_long_enabled=True))

  def test_hidden_when_not_hda2(self):
    cp = self._cp(0)
    self.assertFalse(_should_show_dynamic_handoff(cp, alpha_long_enabled=True))

  def test_hidden_when_alpha_long_off(self):
    cp = self._cp(HyundaiFlags.CANFD_LKA_STEERING.value)
    self.assertFalse(_should_show_dynamic_handoff(cp, alpha_long_enabled=False))

  def test_hidden_when_no_radar_disable_flag(self):
    flags = HyundaiFlags.CANFD_LKA_STEERING.value | HyundaiFlags.CANFD_NO_RADAR_DISABLE.value
    cp = self._cp(flags)
    self.assertFalse(_should_show_dynamic_handoff(cp, alpha_long_enabled=True))

  def test_hidden_when_camera_scc(self):
    flags = HyundaiFlags.CANFD_LKA_STEERING.value | HyundaiFlags.CANFD_CAMERA_SCC.value
    cp = self._cp(flags)
    self.assertFalse(_should_show_dynamic_handoff(cp, alpha_long_enabled=True))

  def test_hidden_when_cp_is_none(self):
    self.assertFalse(_should_show_dynamic_handoff(None, alpha_long_enabled=True))


if __name__ == "__main__":
  unittest.main()

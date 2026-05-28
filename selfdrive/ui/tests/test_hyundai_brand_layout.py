#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Tests for the Hyundai brand layout visibility logic.

These tests exercise the pure-function visibility predicate
(should_show_dynamic_handoff) without spinning up the UI stack.

Strategy: load hyundai.py directly via importlib, bypassing package
__init__ files that pull in raylib/capnp/Qt. All transitive imports are
satisfied with minimal stubs registered in sys.modules before the load.
"""
import abc
import importlib.util
import os
import sys
import types
import unittest
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# HyundaiFlags integer literal values (from opendbc/car/hyundai/values.py).
# Reproduced here so the test has no runtime dependency on opendbc/capnp.
# ---------------------------------------------------------------------------
_CANFD_LKA_STEERING = 1         # HyundaiFlags.CANFD_LKA_STEERING
_CANFD_CAMERA_SCC = 2 ** 3      # HyundaiFlags.CANFD_CAMERA_SCC
_CANFD_NO_RADAR_DISABLE = 2 ** 20  # HyundaiFlags.CANFD_NO_RADAR_DISABLE


def _build_stubs():
  """Register lightweight stubs for modules that require capnp / UI runtime."""
  # opendbc stubs (capnp not available in dev env)
  for name in (
    "capnp",
    "opendbc",
    "opendbc.car",
    "opendbc.car.structs",
    "opendbc.car.uds",
    "opendbc.car.can_definitions",
    "opendbc.car.docs_definitions",
    "opendbc.car.common",
    "opendbc.car.common.conversions",
    "opendbc.car.fw_query_definitions",
    "opendbc.sunnypilot",
    "opendbc.sunnypilot.car",
    "opendbc.sunnypilot.car.hyundai",
    "opendbc.sunnypilot.car.hyundai.values",
    "opendbc.car.hyundai",
    "opendbc.car.hyundai.values",
  ):
    if name not in sys.modules:
      sys.modules[name] = types.ModuleType(name)

  # Provide the HyundaiFlags mock and car-list stubs used by hyundai.py
  hf = MagicMock()
  hf.CANFD_LKA_STEERING = _CANFD_LKA_STEERING
  hf.CANFD_CAMERA_SCC = _CANFD_CAMERA_SCC
  hf.CANFD_NO_RADAR_DISABLE = _CANFD_NO_RADAR_DISABLE
  values_mod = sys.modules["opendbc.car.hyundai.values"]
  values_mod.HyundaiFlags = hf
  values_mod.CAR = MagicMock()
  values_mod.CANFD_UNSUPPORTED_LONGITUDINAL_CAR = set()
  values_mod.UNSUPPORTED_LONGITUDINAL_CAR = set()

  # openpilot UI stubs
  for name in (
    "openpilot",
    "openpilot.selfdrive",
    "openpilot.selfdrive.ui",
    "openpilot.selfdrive.ui.ui_state",
    "openpilot.selfdrive.ui.sunnypilot",
    "openpilot.selfdrive.ui.sunnypilot.layouts",
    "openpilot.selfdrive.ui.sunnypilot.layouts.settings",
    "openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle",
    "openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands",
    "openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base",
    "openpilot.system",
    "openpilot.system.ui",
    "openpilot.system.ui.lib",
    "openpilot.system.ui.lib.multilang",
    "openpilot.system.ui.sunnypilot",
    "openpilot.system.ui.sunnypilot.widgets",
    "openpilot.system.ui.sunnypilot.widgets.list_view",
  ):
    if name not in sys.modules:
      sys.modules[name] = types.ModuleType(name)

  sys.modules["openpilot.system.ui.lib.multilang"].tr = lambda s: s

  lv = sys.modules["openpilot.system.ui.sunnypilot.widgets.list_view"]
  lv.multiple_button_item_sp = MagicMock(return_value=MagicMock())
  lv.toggle_item_sp = MagicMock(return_value=MagicMock())

  sys.modules["openpilot.selfdrive.ui.ui_state"].ui_state = MagicMock()

  # Minimal BrandSettings ABC so HyundaiSettings can inherit from it
  class _BrandSettings(abc.ABC):
    def __init__(self):
      self.items = []

    @abc.abstractmethod
    def update_settings(self):
      pass

  sys.modules["openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base"].BrandSettings = _BrandSettings


_build_stubs()


def _load_hyundai_module():
  """Load hyundai.py directly, bypassing package __init__ files."""
  path = os.path.join(
    os.path.dirname(__file__),
    "..", "sunnypilot", "layouts", "settings", "vehicle", "brands", "hyundai.py",
  )
  path = os.path.abspath(path)
  spec = importlib.util.spec_from_file_location("_hyundai_brand_layout", path)
  mod = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(mod)
  return mod


_hyundai_mod = _load_hyundai_module()
should_show_dynamic_handoff = _hyundai_mod.should_show_dynamic_handoff


class TestDynamicRadarHandoffVisibility(unittest.TestCase):
  """Visibility predicate tests for the Dynamic Radar Handoff sub-toggle."""

  def _cp(self, flags: int):
    cp = MagicMock()
    cp.flags = flags
    return cp

  def test_visible_when_all_conditions_met(self):
    """Toggle is visible: HDA II set, radar-disable supported, not camera SCC, alpha long on."""
    cp = self._cp(_CANFD_LKA_STEERING)
    self.assertTrue(should_show_dynamic_handoff(cp, alpha_long_enabled=True))

  def test_hidden_when_not_hda2(self):
    """Toggle is hidden when CANFD_LKA_STEERING flag is absent (not HDA II)."""
    cp = self._cp(0)
    self.assertFalse(should_show_dynamic_handoff(cp, alpha_long_enabled=True))

  def test_hidden_when_alpha_long_off(self):
    """Toggle is hidden when alpha longitudinal is not enabled."""
    cp = self._cp(_CANFD_LKA_STEERING)
    self.assertFalse(should_show_dynamic_handoff(cp, alpha_long_enabled=False))

  def test_hidden_when_no_radar_disable_flag(self):
    """Toggle is hidden when CANFD_NO_RADAR_DISABLE is set (radar disable not supported)."""
    flags = _CANFD_LKA_STEERING | _CANFD_NO_RADAR_DISABLE
    cp = self._cp(flags)
    self.assertFalse(should_show_dynamic_handoff(cp, alpha_long_enabled=True))

  def test_hidden_when_camera_scc(self):
    """Toggle is hidden when CANFD_CAMERA_SCC is set."""
    flags = _CANFD_LKA_STEERING | _CANFD_CAMERA_SCC
    cp = self._cp(flags)
    self.assertFalse(should_show_dynamic_handoff(cp, alpha_long_enabled=False))

  def test_hidden_when_cp_is_none(self):
    """Toggle is hidden when CP is not yet available."""
    self.assertFalse(should_show_dynamic_handoff(None, alpha_long_enabled=True))


if __name__ == "__main__":
  unittest.main()

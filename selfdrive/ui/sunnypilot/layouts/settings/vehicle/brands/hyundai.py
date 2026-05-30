"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp, toggle_item_sp
from opendbc.car.hyundai.values import CAR, HyundaiFlags, UNSUPPORTED_LONGITUDINAL_CAR

DYNAMIC_RADAR_HANDOFF_DESCRIPTION = (
  "Restores stock SCC and AEB when sunnypilot is disengaged. AEB may not re-arm reliably after disengagement; " +
  "stock behavior is hardware-dependent. Out of scope: stock AEB is not preserved while sunnypilot is engaged."
)


def should_show_dynamic_handoff(cp, alpha_long_enabled: bool) -> bool:
  """Return True if the Dynamic Radar Handoff toggle should be visible.

  Conditions (all must be true):
  - HDA II detected (CANFD_LKA_STEER_MSG flag set)
  - Radar disable is supported on this platform (CANFD_NO_RADAR_DISABLE NOT set)
  - Not camera SCC (CANFD_CAMERA_SCC NOT set)
  - Alpha Longitudinal is enabled
  """
  if cp is None:
    return False
  flags = cp.flags
  is_hda2 = bool(flags & HyundaiFlags.CANFD_LKA_STEER_MSG)
  radar_disable_supported = not bool(flags & HyundaiFlags.CANFD_NO_RADAR_DISABLE)
  not_camera_scc = not bool(flags & HyundaiFlags.CANFD_CAMERA_SCC)
  return is_hda2 and radar_disable_supported and not_camera_scc and alpha_long_enabled


class HyundaiSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.alpha_long_available = False

    tuning_texts = [tr("Off"), tr("Dynamic"), tr("Predictive")]
    self.longitudinal_tuning_item = multiple_button_item_sp(tr("Custom Longitudinal Tuning"), "", tuning_texts,
                                                            button_width=300, callback=self._on_tuning_selected,
                                                            param="HyundaiLongitudinalTuning", inline=False)

    self.dynamic_radar_handoff_item = toggle_item_sp(
      lambda: tr("Dynamic Radar Handoff"),
      description=lambda: tr(DYNAMIC_RADAR_HANDOFF_DESCRIPTION),
      initial_state=ui_state.params.get_bool("DynamicRadarHandoffEnabled"),
      callback=self._on_dynamic_radar_handoff_toggled,
    )

    self.items = [self.longitudinal_tuning_item, self.dynamic_radar_handoff_item]

  @staticmethod
  def _on_tuning_selected(index):
    ui_state.params.put("HyundaiLongitudinalTuning", index)

  @staticmethod
  def _on_dynamic_radar_handoff_toggled(state: bool):
    ui_state.params.put_bool("DynamicRadarHandoffEnabled", state)

  def update_settings(self):
    self.alpha_long_available = False
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      platform = bundle.get("platform")
      self.alpha_long_available = CAR[platform] not in set().union(*UNSUPPORTED_LONGITUDINAL_CAR.values())
    elif ui_state.CP is not None:
      self.alpha_long_available = ui_state.CP.alphaLongitudinalAvailable

    tuning_param = int(ui_state.params.get("HyundaiLongitudinalTuning") or "0")
    long_enabled = ui_state.has_longitudinal_control

    long_tuning_descs = [
      tr("Your vehicle will use the Default longitudinal tuning."),
      tr("Your vehicle will use the Dynamic longitudinal tuning."),
      tr("Your vehicle will use the Predictive longitudinal tuning."),
    ]
    long_tuning_desc = long_tuning_descs[tuning_param] if tuning_param < len(long_tuning_descs) else long_tuning_descs[0]

    longitudinal_tuning_disabled = not ui_state.is_offroad() or not long_enabled
    if longitudinal_tuning_disabled:
      if not ui_state.is_offroad():
        long_tuning_desc = tr("This feature is unavailable while the car is onroad.")
      elif not long_enabled:
        long_tuning_desc = tr("This feature is unavailable because sunnypilot Longitudinal Control (Alpha) is not enabled.")

    self.longitudinal_tuning_item.action_item.set_enabled(not longitudinal_tuning_disabled)
    self.longitudinal_tuning_item.set_description(long_tuning_desc)
    self.longitudinal_tuning_item.show_description(True)
    self.longitudinal_tuning_item.action_item.set_selected_button(tuning_param)
    self.longitudinal_tuning_item.set_visible(self.alpha_long_available)

    alpha_long_enabled = ui_state.params.get_bool("AlphaLongitudinalEnabled")
    self.dynamic_radar_handoff_item.set_visible(
      should_show_dynamic_handoff(ui_state.CP, alpha_long_enabled)
    )

import time
import numpy as np
from dataclasses import dataclass

from opendbc.car import DT_CTRL, structs
from opendbc.car.hyundai.values import HyundaiFlags, CarControllerParams
from opendbc.sunnypilot.car.hyundai.longitudinal_config import Cartuning
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.interpolation_utils import catmull_rom_interp

LongCtrlState = structs.CarControl.Actuators.LongControlState

class JerkOutput:
  def __init__(self, jerk_upper, jerk_lower):
    self.jerk_upper = jerk_upper
    self.jerk_lower = jerk_lower


@dataclass
class LongitudinalTuningState:
  accel_last: float = 0.0
  accel_last_jerk: float = 0.0
  jerk: float = 0.0


@dataclass
class LongitudinalState:
  accel: float = 0.0
  jerk: JerkOutput | None = None


class LongitudinalTuningController:
  """Longitudinal tuning methodology for HKG"""
  def __init__(self, CP: structs.CarParams) -> None:
    self.CP = CP
    self.state = LongitudinalTuningState()
    self.car_config = Cartuning.get_car_config(self.CP)
    self.DT_CTRL = DT_CTRL
    self.jerk_upper = 0.0
    self.jerk_lower = 0.0
    self.last_decel_time = 0.0
    self.min_cancel_delay = 0.1

  def make_jerk(self, CS: structs.CarState) -> float:
    # Handle cancel state to prevent cruise fault
    if CS.out.brakePressed:
      self.state.accel_last_jerk = 0.0
      self.state.jerk = 0.0
      self.jerk_upper = 0.0
      self.jerk_lower = 0.0
      return 0.0

    current_accel = CS.out.aEgo
    self.state.jerk = (current_accel - self.state.accel_last_jerk) / 0.05     # DT_MDL == driving model which equals 0.05
    self.state.accel_last_jerk = current_accel

    # Jerk is calculated using current accel - last accel divided by ΔT (delta time). It doesn't make sense to use planned accel.

    base_jerk = self.state.jerk
    xp = np.array([-5.0, -3.0, -1.0, 1.0, 2.0, 3.0])
    fp = np.array([-4.0, -2.0, -0.5, 0.0, 1.0, 2.0])
    self.state.jerk = catmull_rom_interp(base_jerk, xp, fp)

    # Jerk is limited by the following conditions imposed by ISO 15622:2018
    velocity = CS.out.vEgo
    if velocity < 5.0:
      decel_jerk_max = self.car_config.jerk_limits[1]
    elif velocity > 20.0:
      decel_jerk_max = 2.5
    else:
      decel_jerk_max = 5.83 - (velocity / 6)
    accel_jerk_max = self.car_config.jerk_limits[2]

    if self.CP.flags & HyundaiFlags.CANFD.value:
      self.jerk_upper = min(max(self.car_config.jerk_limits[0], self.state.jerk * 2.0), accel_jerk_max)
      self.jerk_lower = min(max(self.car_config.jerk_limits[0], -self.state.jerk * 4.0), decel_jerk_max)
    else:
      self.jerk_upper = min(max(self.car_config.jerk_limits[0], self.state.jerk * 2.0), accel_jerk_max)
      self.jerk_lower = min(max(self.car_config.jerk_limits[0], -self.state.jerk * 2.0), decel_jerk_max)

    return self.state.jerk

  def handle_cruise_cancel(self, CS: structs.CarState):
    """Handle cruise control cancel to prevent faults."""
    if CS.out.brakePressed:
      self.state.accel_last = 0.0
      return True
    return False

  def calculate_limited_accel(self, actuators: structs.CarControl.Actuators, CS: structs.CarState) -> float:
    """Adaptive acceleration limiting."""
    if self.handle_cruise_cancel(CS):
      return actuators.accel
    self.make_jerk(CS)
    target_accel = actuators.accel

    # Normal operation = above ~29mph also known as 13 m/s
    if CS.out.vEgo > 13.0 and target_accel < 0.01:
      brake_ratio = np.clip(abs(target_accel / self.car_config.accel_limits[0]), 0.0, 1.0)
      # Array comes from longitudinal_config.py, 1.0 = -3.5 accel, which will never be less than -3.5 EVER
      accel_rate_down = self.DT_CTRL * catmull_rom_interp(brake_ratio,
                                                          np.array([0.25, 0.5, 0.75, 1.0]),
                                                          np.array(self.car_config.brake_response))
      accel = max(target_accel, self.state.accel_last - accel_rate_down)
    else:
      accel = actuators.accel

    target_accel = accel + (target_accel - self.state.accel_last)
    accel = target_accel
    self.state.accel_last = accel
    return accel

  def calculate_accel(self, actuators: structs.CarControl.Actuators, CS: structs.CarState) -> float:
    """Calculate acceleration with cruise control status handling."""
    if self.handle_cruise_cancel(CS):
      return 0.0
    accel = self.calculate_limited_accel(actuators, CS)
    return float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

  def apply_tune(self, CP: structs.CarParams) -> None:
    config = self.car_config
    CP.vEgoStopping = config.vego_stopping
    CP.vEgoStarting = config.vego_starting
    CP.stoppingDecelRate = config.stopping_decel_rate
    CP.startAccel = config.start_accel
    CP.startingState = True
    CP.longitudinalActuatorDelay = 0.5


class LongitudinalController:
  """Longitudinal controller which gets injected into CarControllerParams."""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP = None) -> None:
    self.CP = CP
    self.CP_SP = CP_SP
    self.tuning = LongitudinalTuningController(CP) if self.CP_SP is not None \
                  and (self.CP_SP.flags & HyundaiFlagsSP.LONGTUNING) else None
    self.state = LongitudinalState()
    self.jerk_upper= 0.0
    self.jerk_lower = 0.0
    self.stop_req_transition_time = 0.0     # Time when StopReq changed from 1 to 0 (note: StopReq uses stopping)
    self.standstill_delay = 0.9             # Delay in which commands from model are not sent
    self.prev_stop_req = 1                  # 1 == stopped

  def apply_tune(self, CP: structs.CarParams):
    if self.CP_SP is not None and (self.CP_SP.flags & HyundaiFlagsSP.LONGTUNING):
      self.tuning.apply_tune(CP)
    else:
      CP.vEgoStopping = 0.5
      CP.vEgoStarting = 0.1
      CP.startingState = True
      CP.startAccel = 1.0
      CP.longitudinalActuatorDelay = 0.5

  def get_jerk(self) -> JerkOutput:
    if self.tuning is not None:
      return JerkOutput(
        self.tuning.jerk_upper,
        self.tuning.jerk_lower,
      )
    else:
      return JerkOutput(
        self.jerk_upper,
        self.jerk_lower,
      )

  def calculate_and_get_jerk(self, CS: structs.CarState,
                             long_control_state: LongCtrlState) -> JerkOutput:
    """Calculate jerk based on tuning and return JerkOutput."""
    if self.tuning is not None:
      self.tuning.make_jerk(CS)
    else:
      jerk_limit = 3.0 if long_control_state == LongCtrlState.pid else 1.0

      self.jerk_upper = jerk_limit
      self.jerk_lower = jerk_limit
    return self.get_jerk()

  def calculate_accel(self, actuators: structs.CarControl.Actuators, CS: structs.CarState,
                      CP: structs.CarParams) -> float:
    """Calculate acceleration based on tuning and return the value."""
    if CP.flags & HyundaiFlagsSP.LONGTUNING_BRAKING and self.tuning is not None:
      accel = self.tuning.calculate_accel(actuators, CS)
    else:
      accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    return accel

  def update(self, CC: structs.CarControl, CS: structs.CarState, CP: structs.CarParams) -> None:
    """Inject Longitudinal Controls for HKG Vehicles."""
    actuators = CC.actuators
    long_control_state = actuators.longControlState
    j = self.calculate_and_get_jerk(CS, long_control_state)
    self.state.jerk = j  # Store the JerkOutput object from our def.
    stopping = long_control_state == LongCtrlState.stopping
    current_stop_req = 1 if stopping else 0
    stop_req_transition = (self.prev_stop_req == 1 and current_stop_req == 0)

    if stop_req_transition:
      self.stop_req_transition_time = float(time.monotonic())

    # Time since transition
    time_since_transition = float(time.monotonic()) - self.stop_req_transition_time
    self.prev_stop_req = current_stop_req

    # Check if we should force zero accel
    force_zero = self.tuning is not None and (stop_req_transition or (current_stop_req == 0 and
                                              time_since_transition < self.standstill_delay))

    if force_zero:
      # Force zero acceleration during standstill delay of 0.9 seconds
      self.state.accel = 0.0
      self.jerk_upper = self.jerk_lower = 0.0
    else:
      # Not transitioning from stopping
      self.state.accel = self.calculate_accel(actuators, CS, CP)
      self.jerk_upper = j.jerk_upper
      self.jerk_lower = j.jerk_lower

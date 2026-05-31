from cereal import log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.selfdrived import (
  mute_can_loss_at_shutdown,
  CAN_LOSS_SHUTDOWN_EVENTS,
  SHUTDOWN_MUTE_MAX_SPEED,
)

EventName = log.OnroadEvent.EventName

# A non-CAN event that must never be touched by the shutdown mute.
KEEP_EVENT = EventName.wrongGear

# Defaults describe the observed end-of-drive burst: openpilot long-disengaged,
# MADS inactive, the car's CAN bus gone, vehicle coasting to a stop.
SHUTDOWN = dict(enabled=False, mads_active=False, can_valid=False, can_timeout=True, v_ego=1.01)


def make_events(names):
  events = Events()
  for n in names:
    events.add(n)
  return events


class TestShutdownCanMute:
  def test_mutes_full_burst_at_shutdown(self):
    events = make_events([*CAN_LOSS_SHUTDOWN_EVENTS, KEEP_EVENT])
    muted = mute_can_loss_at_shutdown(events, **SHUTDOWN)
    assert muted
    for e in CAN_LOSS_SHUTDOWN_EVENTS:
      assert e not in events.names
    assert KEEP_EVENT in events.names

  def test_not_muted_when_engaged(self):
    events = make_events(list(CAN_LOSS_SHUTDOWN_EVENTS))
    muted = mute_can_loss_at_shutdown(events, **{**SHUTDOWN, "enabled": True})
    assert not muted
    for e in CAN_LOSS_SHUTDOWN_EVENTS:
      assert e in events.names

  def test_not_muted_when_mads_active(self):
    events = make_events(list(CAN_LOSS_SHUTDOWN_EVENTS))
    muted = mute_can_loss_at_shutdown(events, **{**SHUTDOWN, "mads_active": True})
    assert not muted
    for e in CAN_LOSS_SHUTDOWN_EVENTS:
      assert e in events.names

  def test_not_muted_while_moving(self):
    # genuine mid-drive CAN dropout (vEgo frozen high) must still alert
    events = make_events(list(CAN_LOSS_SHUTDOWN_EVENTS))
    muted = mute_can_loss_at_shutdown(events, **{**SHUTDOWN, "v_ego": 25.0})
    assert not muted
    for e in CAN_LOSS_SHUTDOWN_EVENTS:
      assert e in events.names

  def test_not_muted_when_can_healthy(self):
    events = make_events(list(CAN_LOSS_SHUTDOWN_EVENTS))
    muted = mute_can_loss_at_shutdown(events, **{**SHUTDOWN, "can_valid": True, "can_timeout": False})
    assert not muted
    for e in CAN_LOSS_SHUTDOWN_EVENTS:
      assert e in events.names

  def test_can_timeout_alone_triggers(self):
    events = make_events([EventName.canBusMissing])
    assert mute_can_loss_at_shutdown(events, **{**SHUTDOWN, "can_valid": True, "can_timeout": True})
    assert EventName.canBusMissing not in events.names

  def test_can_invalid_alone_triggers(self):
    events = make_events([EventName.canError])
    assert mute_can_loss_at_shutdown(events, **{**SHUTDOWN, "can_valid": False, "can_timeout": False})
    assert EventName.canError not in events.names

  def test_boundary_speed_not_muted(self):
    events = make_events(list(CAN_LOSS_SHUTDOWN_EVENTS))
    muted = mute_can_loss_at_shutdown(events, **{**SHUTDOWN, "v_ego": SHUTDOWN_MUTE_MAX_SPEED})
    assert not muted

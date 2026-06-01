from cereal import log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.selfdrived import main_button_engages_op

EventName = log.OnroadEvent.EventName

# All gates satisfied, on the cruise-available rising edge (the main-button press).
ENGAGE = dict(op_long=True, unified_engagement=True, main_allowed=True,
              cruise_available=True, cruise_available_prev=False)


def make_events():
  return Events()


class TestMainButtonEngagesOp:
  def test_engages_on_main_rising_edge(self):
    events = make_events()
    assert main_button_engages_op(events, **ENGAGE)
    assert EventName.buttonEnable in events.names

  def test_no_engage_when_not_op_long(self):
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "op_long": False})
    assert EventName.buttonEnable not in events.names

  def test_no_engage_without_unified_engagement(self):
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "unified_engagement": False})
    assert EventName.buttonEnable not in events.names

  def test_no_engage_without_main_allowed(self):
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "main_allowed": False})
    assert EventName.buttonEnable not in events.names

  def test_no_engage_without_rising_edge(self):
    # cruise already available last cycle -> not a fresh main press
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "cruise_available_prev": True})
    assert EventName.buttonEnable not in events.names

  def test_no_engage_when_cruise_unavailable(self):
    events = make_events()
    assert not main_button_engages_op(events, **{**ENGAGE, "cruise_available": False})
    assert EventName.buttonEnable not in events.names

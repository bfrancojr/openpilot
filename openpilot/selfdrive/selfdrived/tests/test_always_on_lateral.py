from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import Events, ET, EVENTS, NormalPermanentAlert
from openpilot.selfdrive.selfdrived.state import always_on_lateral_allowed

EventName = log.OnroadEvent.EventName


def make_event(event_types):
  EVENTS[0] = {ev: NormalPermanentAlert("alert") for ev in event_types}
  return 0


class TestAlwaysOnLateral(OpenpilotTestCase):
  def setup_method(self):
    self.events = Events()

  def test_allowed_when_engageable(self):
    assert always_on_lateral_allowed(self.events, cruise_available=True)

  def test_warnings_and_overrides_do_not_block(self):
    self.events.add(make_event([ET.WARNING, ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL, ET.PERMANENT, ET.PRE_ENABLE]))
    assert always_on_lateral_allowed(self.events, cruise_available=True)

  def test_user_disable_alone_does_not_block(self):
    # pcmDisable is USER_DISABLE only and is present whenever stock cruise isn't set,
    # which is exactly the situation always-on lateral exists for
    self.events.add(make_event([ET.USER_DISABLE]))
    assert always_on_lateral_allowed(self.events, cruise_available=True)

  def test_cruise_main_off_blocks(self):
    assert not always_on_lateral_allowed(self.events, cruise_available=False)

  def test_blocking_event_types(self):
    for et in (ET.NO_ENTRY, ET.SOFT_DISABLE, ET.IMMEDIATE_DISABLE):
      self.events.clear()
      self.events.add(make_event([et, ET.WARNING]))
      assert not always_on_lateral_allowed(self.events, cruise_available=True), et

  def test_steer_while_braking_exempts_only_the_brake(self):
    self.events.add(EventName.pedalPressed)
    assert not always_on_lateral_allowed(self.events, cruise_available=True)
    assert always_on_lateral_allowed(self.events, cruise_available=True, steer_while_braking=True)
    # anything else that blocks entry still blocks
    self.events.add(EventName.doorOpen)
    assert not always_on_lateral_allowed(self.events, cruise_available=True, steer_while_braking=True)

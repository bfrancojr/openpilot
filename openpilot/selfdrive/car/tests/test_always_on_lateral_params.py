from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.structs import car
from opendbc.car.toyota.values import CAR as TOYOTA, ToyotaSafetyFlags
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.card import apply_always_on_lateral


def get_params(platform):
  CP = interfaces[platform].get_non_essential_params(platform)
  CP.alternativeExperience = 0
  return CP


def roundtrip(CP):
  # card publishes CP through capnp; a CP that can't serialize never reaches selfdrived
  data = CP.to_bytes()
  CP.clear_write_flag()  # tests serialize the same builder twice
  with car.CarParams.from_bytes(data) as parsed:
    return parsed.alternativeExperience, parsed.safetyConfigs[0].safetyParam


class TestAlwaysOnLateralParams(OpenpilotTestCase):
  def test_enabled_on_supported_toyota(self):
    CP = get_params(TOYOTA.TOYOTA_RAV4_TSS2)
    base_param = CP.safetyConfigs[0].safetyParam
    assert apply_always_on_lateral(CP, always_on=True, while_braking=False)
    alt_exp, safety_param = roundtrip(CP)
    assert alt_exp == ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL
    assert safety_param == base_param | ToyotaSafetyFlags.ACC_MAIN_ON.value

  def test_while_braking_adds_second_bit(self):
    CP = get_params(TOYOTA.TOYOTA_RAV4_TSS2)
    assert apply_always_on_lateral(CP, always_on=True, while_braking=True)
    alt_exp, safety_param = roundtrip(CP)
    assert alt_exp == ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL | ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL_WHILE_BRAKING
    assert safety_param & ToyotaSafetyFlags.ACC_MAIN_ON.value

  def test_toggle_off_leaves_params_untouched(self):
    CP = get_params(TOYOTA.TOYOTA_RAV4_TSS2)
    base = roundtrip(CP)
    assert not apply_always_on_lateral(CP, always_on=False, while_braking=True)
    assert roundtrip(CP) == base

  def test_unsupported_dsu_toyota_is_refused(self):
    CP = get_params(TOYOTA.LEXUS_IS)
    base = roundtrip(CP)
    assert not apply_always_on_lateral(CP, always_on=True, while_braking=True)
    assert roundtrip(CP) == base

  def test_non_toyota_is_refused(self):
    CP = get_params(HONDA.HONDA_CIVIC)
    base = roundtrip(CP)
    assert not apply_always_on_lateral(CP, always_on=True, while_braking=True)
    assert roundtrip(CP) == base

  def test_passive_is_refused(self):
    CP = get_params(TOYOTA.TOYOTA_RAV4_TSS2)
    CP.passive = True
    base = roundtrip(CP)
    assert not apply_always_on_lateral(CP, always_on=True, while_braking=False)
    assert roundtrip(CP) == base

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized

from openpilot.cereal import log
from opendbc.car.structs import car
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.toyota.values import CAR as TOYOTA
from opendbc.car.nissan.values import CAR as NISSAN
from opendbc.car.gm.values import CAR as GM
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle


class TestLatControl(OpenpilotTestCase):

  @parameterized.expand([(HONDA.HONDA_CIVIC, LatControlPID), (TOYOTA.TOYOTA_RAV4, LatControlTorque),
                         (NISSAN.NISSAN_LEAF, LatControlAngle), (GM.CHEVROLET_BOLT_EUV, LatControlTorque)])
  def test_saturation(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringPressed = False

    params = log.VehicleParameters.new_message()

    # Saturate for curvature limited and controller limited
    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, True, 0.2)
    assert lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, False, 0.2)
    assert not lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 1, False, 0.2)
    assert lac_log.saturated

  def test_friction_boost(self):
    car_name = TOYOTA.TOYOTA_SIENNA
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)
    params = log.VehicleParameters.new_message()
    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringPressed = False
    full_friction = CP.lateralTuning.torque.friction * CP.lateralTuning.torque.latAccelFactor

    def feedforward(boost, lat_accel_error):
      controller = LatControlTorque(CP.as_reader(), CI, DT_CTRL)
      controller.set_friction_boost(boost)
      # steady request with the wheel straight: once the request buffer fills, the error is the request and the jerk is 0
      for _ in range(200):
        _, _, lac_log = controller.update(True, CS, VM, params, False, lat_accel_error / CS.vEgo ** 2, False, 0.2)
      return lac_log.f - lat_accel_error

    # halfway to the default threshold, the boost gives the full friction push instead of half of it
    assert abs(feedforward(False, 0.1) - full_friction / 2) < 1e-3
    assert abs(feedforward(True, 0.1) - full_friction) < 1e-3
    # past both thresholds they agree
    assert abs(feedforward(True, 0.3) - feedforward(False, 0.3)) < 1e-6

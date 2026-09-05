from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL
import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
from openpilot.selfdrive.controls.lib.lane_offset import LaneOffsetController, MAX_CURVATURE, MAX_LAT_ACCEL, MIN_LOOKAHEAD
from openpilot.selfdrive.modeld.constants import ModelConstants

SETTLE_STEPS = int(5.0 / DT_CTRL)  # 10 filter time constants


def make_model(y_left, y_right, probs=(0.0, 0.9, 0.9, 0.0), lane_change=False):
  md = messaging.new_message('modelV2').modelV2
  md.init('laneLines', 4)
  for i, y in enumerate((y_left - 3.0, y_left, y_right, y_right + 3.0)):
    md.laneLines[i].x = list(ModelConstants.X_IDXS)
    md.laneLines[i].y = [float(y)] * ModelConstants.IDX_N
  md.laneLineProbs = list(probs)
  if lane_change:
    md.meta.laneChangeState = log.LaneChangeState.laneChangeStarting
  return md


def make_cs(v_ego=20.0, left_blinker=False, right_blinker=False, steering_pressed=False):
  cs = car.CarState.new_message()
  cs.vEgo = v_ego
  cs.leftBlinker = left_blinker
  cs.rightBlinker = right_blinker
  cs.steeringPressed = steering_pressed
  return cs


def settled(ctrl, md, cs, target=0.0, active=True, steps=SETTLE_STEPS):
  k = 0.0
  for _ in range(steps):
    k = ctrl.update(active, target, md, cs)
  return k


class TestLaneOffset(OpenpilotTestCase):
  def setup_method(self):
    self.ctrl = LaneOffsetController(DT_CTRL)

  def test_car_left_of_centre_steers_right(self):
    # lines at -1.2 / +1.8: the lane centre is 0.3 m to the right, so the car sits 0.3 m left
    k = settled(self.ctrl, make_model(-1.2, 1.8), make_cs(20.0))
    expected = 2.0 * 0.3 / (20.0 * 2.0) ** 2  # positive curvature = right turn
    assert abs(k - expected) < 1e-5, (k, expected)
    assert k > 0

  def test_sign_matches_model_curvature_convention(self):
    # The model frame is x forward, y right, z down: a right turn has positive yaw, and the upstream helper that
    # turns the model's plan into a desired curvature returns a positive value for it (verified against logged
    # routes: desiredCurvature shares the sign of the lane centre ahead in every frame). Moving the car right
    # must therefore add curvature of the same sign. Shipped inverted once (2026-09-05); keep this pinned.
    t_idxs = np.array(ModelConstants.T_IDXS)
    yaw_rates = np.full(len(t_idxs), 0.05)  # rad/s, turning right
    right_turn = get_curvature_from_plan(yaw_rates * t_idxs, yaw_rates, t_idxs, 20.0, 0.2)
    assert right_turn > 0
    k = settled(LaneOffsetController(DT_CTRL), make_model(-1.2, 1.8), make_cs(20.0))  # car left of centre: move right
    assert np.sign(k) == np.sign(right_turn)

  def test_target_is_right_of_centre_positive(self):
    centred = make_model(-1.5, 1.5)
    assert settled(self.ctrl, centred, make_cs(), target=0.0) == 0.0
    assert settled(LaneOffsetController(DT_CTRL), centred, make_cs(), target=0.3) > 3e-4   # move right: positive curvature
    assert settled(LaneOffsetController(DT_CTRL), centred, make_cs(), target=-0.3) < -3e-4  # move left: negative curvature

  def test_target_reached_means_no_correction(self):
    # car 0.25 m right of centre, target 0.25 m right
    assert settled(self.ctrl, make_model(-1.75, 1.25), make_cs(), target=0.25) == 0.0

  def test_inactive_resets_immediately(self):
    settled(self.ctrl, make_model(-1.2, 1.8), make_cs())
    assert self.ctrl.update(False, 0.0, make_model(-1.2, 1.8), make_cs()) == 0.0
    assert self.ctrl.filter.x == 0.0

  def test_gating(self):
    off_centre = make_model(-1.2, 1.8)
    cases = {
      'low line confidence': (make_model(-1.2, 1.8, probs=(0.0, 0.4, 0.9, 0.0)), make_cs()),
      'left blinker': (off_centre, make_cs(left_blinker=True)),
      'right blinker': (off_centre, make_cs(right_blinker=True)),
      'driver steering': (off_centre, make_cs(steering_pressed=True)),
      'low speed': (off_centre, make_cs(v_ego=4.0)),
      'model lane change': (make_model(-1.2, 1.8, lane_change=True), make_cs()),
      'implausible lane width': (make_model(-0.5, 0.5), make_cs()),
      'error too large (not our lane)': (make_model(-3.0, -0.5), make_cs()),
      'missing lines': (messaging.new_message('modelV2').modelV2, make_cs()),
    }
    for name, (md, cs) in cases.items():
      assert settled(LaneOffsetController(DT_CTRL), md, cs) == 0.0, name

  def test_confidence_ramp(self):
    full = settled(LaneOffsetController(DT_CTRL), make_model(-1.2, 1.8), make_cs())
    half = settled(LaneOffsetController(DT_CTRL), make_model(-1.2, 1.8, probs=(0.0, 0.65, 0.9, 0.0)), make_cs())
    assert abs(half - full / 2) < 1e-6

  def test_limits(self):
    far = make_model(-0.6, 2.4)  # car 0.9 m left of centre
    k_slow = settled(LaneOffsetController(DT_CTRL), far, make_cs(v_ego=6.0))
    assert abs(-2.0 * 0.9 / MIN_LOOKAHEAD ** 2) > MAX_CURVATURE  # unclipped it would exceed the cap
    assert abs(k_slow - MAX_CURVATURE) < 1e-5
    k_fast = settled(LaneOffsetController(DT_CTRL), far, make_cs(v_ego=30.0))
    assert abs(k_fast - MAX_LAT_ACCEL / 30.0 ** 2) < 1e-6

  def test_fades_out(self):
    k = settled(self.ctrl, make_model(-1.2, 1.8), make_cs())
    assert abs(k) > 3e-4
    k = settled(self.ctrl, make_model(-1.2, 1.8), make_cs(left_blinker=True))
    assert abs(k) < 2e-6

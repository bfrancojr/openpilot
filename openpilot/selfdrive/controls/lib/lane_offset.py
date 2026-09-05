"""Lane-centre offset correction on top of the model's desired curvature.

The driving model outputs curvature directly and only softly centres the car in its lane:
measured on a TSS2 Sienna (2026-09-04) it held 0.1-0.27 m left of its own lane centre while
the driver preferred 0.25 m right. This adds a small, bounded curvature that steers the car
toward a chosen offset from the centre of the model's inner lane lines. It is inert without
two confident lane lines, below MIN_SPEED, with a blinker on, during a model lane change,
while the driver steers, or when lateral control is off.

Frames: model y is positive to the right of the car, and curvature is positive for a RIGHT turn
(same z-down frame as the model; steeringAngleDeg is the opposite sign). Verified against logged
routes: the model's desiredCurvature shares the sign of the lane centre 40 m ahead in every frame.
"""
from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter

LOOKAHEAD_T = 2.0              # s: the correction aims to close the offset error this far ahead
MIN_LOOKAHEAD = 15.0           # m: bounds the gain at low speed
MIN_SPEED = 5.0                # m/s
MAX_LAT_ACCEL = 0.4            # m/s^2 the correction may add
MAX_CURVATURE = 0.005          # 1/m absolute cap on the correction
LANE_WIDTH_RANGE = (2.4, 4.5)  # m: reject implausible lane-line pairs
MAX_ERROR = 1.0                # m: larger errors mean the lines aren't ours (lane change, merge)
PROB_RAMP = (0.5, 0.8)         # lane-line probability: no correction below, full above
FILTER_RC = 0.5                # s: smooths line noise and fades the correction in and out


def clip(x: float, lo: float, hi: float) -> float:
  return max(lo, min(hi, x))


class LaneOffsetController:
  def __init__(self, dt: float):
    self.filter = FirstOrderFilter(0.0, FILTER_RC, dt)
    self.curvature = 0.0

  @staticmethod
  def offset_error(target: float, model_v2) -> tuple[float, float] | None:
    """How far right the car must move to sit `target` m right of the lane centre, and the
    lane-line confidence in [0, 1]. None when the model's inner lines aren't usable."""
    lines, probs = model_v2.laneLines, model_v2.laneLineProbs
    if len(lines) < 4 or len(probs) < 4 or len(lines[1].y) == 0 or len(lines[2].y) == 0:
      return None
    y_left, y_right = lines[1].y[0], lines[2].y[0]
    width = y_right - y_left
    if not (LANE_WIDTH_RANGE[0] <= width <= LANE_WIDTH_RANGE[1]):
      return None
    centre = (y_left + y_right) / 2.0  # lane centre relative to the car, positive when it is to the right
    error = target + centre            # the car sits at -centre relative to the lane centre
    if abs(error) > MAX_ERROR:
      return None
    confidence = (min(probs[1], probs[2]) - PROB_RAMP[0]) / (PROB_RAMP[1] - PROB_RAMP[0])
    return error, clip(confidence, 0.0, 1.0)

  def update(self, active: bool, target: float, model_v2, CS) -> float:
    """Curvature (1/m, positive right) to add to the model's desired curvature."""
    if not active:
      self.filter.x = 0.0
      self.curvature = 0.0
      return 0.0

    raw = 0.0
    lane_change = model_v2.meta.laneChangeState != log.LaneChangeState.off
    blocked = CS.vEgo < MIN_SPEED or CS.leftBlinker or CS.rightBlinker or CS.steeringPressed or lane_change
    result = None if blocked else self.offset_error(target, model_v2)
    if result is not None:
      error, confidence = result
      lookahead = max(CS.vEgo * LOOKAHEAD_T, MIN_LOOKAHEAD)
      # a lateral shift d over distance L needs curvature 2d/L^2; moving right (+error) is a right (positive) curvature
      raw = 2.0 * error * confidence / lookahead ** 2
      limit = min(MAX_LAT_ACCEL / max(CS.vEgo, MIN_SPEED) ** 2, MAX_CURVATURE)
      raw = clip(raw, -limit, limit)
    self.curvature = self.filter.update(raw)
    return self.curvature

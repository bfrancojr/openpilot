# alka-toyota fork: findings and decisions

Working notes for this fork of openpilot (bfrancojr/openpilot + bfrancojr/opendbc), written so that the reasoning behind the
code survives the chat sessions that produced it. Car: **Toyota Sienna 2018-20 (TSS-P, non-SecOC, fingerprint TOYOTA_SIENNA)**,
device: **comma 4 (mici)**. Dates are 2026. Commit hashes refer to the two forks.

## Branches

| Branch (both forks) | Purpose | State |
|---|---|---|
| `alka-toyota` | Always-on lateral + lane offset correction. Frozen as the device's installation branch. | openpilot `93bfcc38a`, opendbc `e385d87c` |
| `alka-toyota-long` | Everything above plus openpilot longitudinal on TSS-P (DSU unplugged or smartDSU). Where new work lands. | openpilot `513f4da4b`, opendbc `5ea42503` and later |

Install/switch: `installer.comma.ai/bfrancojr/<branch>`, or Settings → Software → target branch. The device builds on the
first boot after a change (no prebuilt marker); Params keys are compiled into the params library, so a new key needs that build.

## 1. Always-on lateral (steer whenever the cruise main switch is on)

Port of dragonpilot's ALKA idea onto upstream master (e3def3769, 2026-09-03). Decisions the owner made: the panda gates on the
stock ACC main switch (not a blanket flag); driver monitoring treats lateral-active as engaged; Toyota TSS-P/TSS2 non-SecOC only.

- Panda (opendbc safety): `ALT_EXP_ALWAYS_ON_LATERAL` (32) and `ALT_EXP_ALWAYS_ON_LATERAL_WHILE_BRAKING` (64);
  `get_lateral_allowed() = controls_allowed || (flag && acc_main_on && (!brake_pressed || while_braking))` in every steering check.
  Toyota reads `PCM_CRUISE_2.MAIN_ON` (0x1D3 bit 15, 33 Hz) only with safety param `ToyotaSafetyFlags.ACC_MAIN_ON` (16<<8).
  rx checksum/timeout failures clear `acc_main_on` like `controls_allowed`. Coverage 100 % on the touched safety files, MISRA clean.
- openpilot: `card.py` decides once at start from the `AlwaysOnLateral` / `AlwaysOnLateralWhileBraking` params and records the
  alternative-experience bits + safety param in CarParams, so panda and selfdrived cannot disagree. `selfdriveState.lateralActive`
  (capnp @14) = engaged-active OR (main on AND no NO_ENTRY/SOFT_DISABLE/IMMEDIATE_DISABLE event; `pedalPressed` exempt when
  steer-while-braking). controlsd steers on it, DM counts it as engaged, UI shows a blue path (`UIStatus.LATERAL`).
- Toggles restart the car processes through `OnroadCycleRequested` (a few seconds, no reboot); grayed out while engaged.
- **Lesson (2026-09-04):** the first on-car build crashed `card` ("openpilot Unavailable / Waiting to start"): `safetyParam |=
  ToyotaSafetyFlags.ACC_MAIN_ON` yields an IntFlag object and pycapnp refuses it. Use `.value`. Any code that writes CarParams
  gets a capnp round-trip test (`test_always_on_lateral_params.py`). Validated on the car the same evening.

## 2. Lane position

**Complaint:** openpilot hugged the left line and drifted to the middle of unmarked roads.

**Measured (2026-09-04, routes 00000007–0a, from the model's own inner lane lines at x = 0; car-frame y is +right, so the
offset (yL+yR)/2 is positive when the car is LEFT of center; filters: both line probs > 0.5, width 2.4–4.5 m, v > 8 m/s, no
blinker; "openpilot" = `carControl.latActive` and not `steeringPressed`):**

| Who steered | Offset from lane center | Time |
|---|---|---|
| openpilot | 0.11 m left mean, 0.17 m median; worst route 0.27 m left | 438 s |
| driver | 0.25 m right | 54 s |

Ruled out: traffic convention (`IsRhdDetected` = 0), mount (centered), calibration (healthy, yaw −0.004 rad), curvature tracking
(mean desired − actual ≤ 3e-4 1/m, mixed sign). The model's own planned path sits where the car is and returns to center only
30 m out: a soft policy, not a control fault. Unmarked roads: too little data to compare.

**Lane offset correction (`openpilot/selfdrive/controls/lib/lane_offset.py`, params `LaneOffsetCorrection`,
`LaneOffsetCm` + = right, live-reloaded every second):** curvature = 2·error·confidence / L², L = max(v·2 s, 15 m), capped at
0.4 m/s² and 0.005 1/m, 0.5 s low-pass; inert without two confident lines, below 5 m/s, with a blinker, during a model lane
change, while the driver steers, when lateral is off, or when a lateral maneuver plan is being replayed. Added before
`clip_curvature`, so all downstream limits and the panda still apply. Replaying the real controlsd over logged routes: toggle off
is bit-identical to upstream over 17,922 frames; toggle on shifts curvature only on lateral-active frames.

**Lesson (2026-09-05):** the first version shipped with the sign inverted. openpilot's curvature is **positive for a RIGHT
turn** (model frame x forward, y right, z down; `steeringAngleDeg` is positive left). Measured: over 5,932 logged frames the
model's desired curvature shares the sign of the lane center 40 m ahead in 100 % of frames and correlates −0.98 with
steering angle. Pinned by `test_sign_matches_model_curvature_convention`. Rule: measure every sign convention from a route
before control code goes to the car.

**What the first drive taught (route 0000000b):** with the wrong sign the car settled 0.35–0.57 m right of center while the
controller pushed 2–4e-4 1/m, i.e. the model resists a displacement at roughly 0.4 m/s² per meter at 25 m/s, comparable to
the controller's gain. Expect about half of a requested offset at highway speed, more at lower speed, until the gain or a
bounded integral term is tuned from a measured drive. Also: the owner remounted the device between the two days; calibrationd
detected the 6.4° pitch / 0.7° yaw change and recalibrated (pitch −0.062 → +0.053 rad, yaw −0.003 → −0.015 rad). With the new
mounting and the correction off, openpilot centers the car (−0.03 m over 25 min). The original complaint is most likely a
mounting artifact; the correction is optional.

## 3. Stop-and-go: openpilot longitudinal on a TSS-P car

**Why the car has none:** on TSS-P the DSU (Driver Support Unit) sends `ACC_CONTROL` (0x343) and the stock radar cruise is
not full-speed-range (cannot be set below 19 mph, drops out at low speed). Logged CarParams on the car:
`openpilotLongitudinalControl = False`, `STOCK_LONGITUDINAL` safety flag, DSU present in the firmware query. openpilot's
"stop and go" needs openpilot to send the ACC commands itself.

**Upstream history:** disconnected-DSU longitudinal removed in commaai/opendbc#2931 (15b8354e, 2025-12-04); the stop-timer
distinction (`NO_STOP_TIMER`) and the standstill handling for TSS-P cars with a stop timer removed in #3076 (1cd92abb,
2026-01-27); smartDSU support removed earlier still. The panda safety side never lost the messages: 0x343 and the static DSU
messages are still in `TOYOTA_COMMON_LONG_TX_MSGS`.

**Restored on `alka-toyota-long` (opendbc 225d987d, 4b8d519d, 5ea42503):**
- `CarParams.enableDsu` moved back out of the capnp `deprecated` group (same ordinal @5).
- Decision at every start from the firmware query and the CAN fingerprint, for cars whose DSU does the longitudinal control
  (not NO_DSU, not UNSUPPORTED_DSU) **and** that carry `NO_STOP_TIMER` (TSS2 configs, Highlander, Prius, Prius V, RAV4H,
  Sienna):
  - DSU answers, no smartDSU → stock longitudinal, unchanged.
  - DSU absent → `enableDsu`: openpilot longitudinal, `STATIC_DSU_MSGS` sent in place of the DSU, `stockAeb` not read.
  - smartDSU (0x2FF on bus 0, DSU present) → `SMART_DSU` flag: openpilot longitudinal, no stand-in messages, AEB still the DSU's.
  - `autoResumeSng` only with `NO_STOP_TIMER`, as before #3076. Other TSS-P cars stay stock: their standstill-request handling
    is not restored (no car to prove it on).
- Pinned in `TestToyotaDisconnectedDsu` (opendbc/car/toyota/tests/test_toyota.py).

**Operational rules:**
- **Reboot after every plug/unplug of the DSU or sDSU.** `CarParamsCache` is cleared only on manager start; within a boot the
  cached firmware list would still say "no DSU".
- Forcing openpilot longitudinal with the DSU connected trips the panda's relay-malfunction check (it hears the DSU's 0x343
  on a message it sends itself) → "Harness Relay Malfunction", immediate disable, no entry. This is by design.
- Unplugged DSU = no stock cruise when openpilot is off and no stock pre-collision braking. The smartDSU keeps AEB. The owner
  ordered a BearTech C-SDSU (tested by the vendor on a 2019 Sienna). **Unverified until installed:** that it emits 0x2FF like
  comma's smartDSU. If it does not, openpilot stays on the stock path while the sDSU blocks the DSU → no cruise at all (safe,
  obvious).
- "Cruise Fault, restart car" on first use = the PCM's low-speed lockout (`PCM_CRUISE_2.LOW_SPEED_LOCKOUT == 2`) not clearing.
  Community fix from 2020: switch the car fully off with the comma still powered, wait one to two minutes, start again.

**First-drive checklist (either hardware path):** CarParams in the rlog shows `openpilotLongitudinalControl`, `enableDsu` or
`flags & 2`; no `relayMalfunction` / `controlsMismatch` events; engages below 19 mph; holds a stop and resumes; then the
longitudinal measurements below.

## 4. Longitudinal quality review (vs. the 2020 Reddit thread "2018 Sienna questions")

redbeards (2020, comma two, openpilot 0.7/0.8): openpilot floors the gas on a set-speed bump, brakes instead of coasting on a
set-speed drop, stock ACC felt better but cuts out below 27 mph. Status on this base:

| 2020 complaint | Current code |
|---|---|
| Mashes gas on set-speed bump | Planner caps cruise accel by speed: `A_CRUISE_MAX_VALS` 1.6/1.2/0.8/0.6 m/s² at 0/10/25/40 m/s, jerk limits (`J_CRUISE_VALS`), Toyota `ACCEL_MAX` 1.5 m/s² (non-TSS2), `ACCEL_MIN` −3.5 |
| Brakes instead of coasting | `A_CRUISE_MIN` −1.2 m/s²; model "allow throttle" → planner coasts at the pitch-fitted coast accel; `PERMIT_BRAKING` handshake (brake only below 0.2 m/s² net request, release above 0.3) |
| No user control | Driving personality (follow distance, jerk cost) |
| Tuning by editing files | Toyota PCM accel PID (feedforward + integral, pitch compensation, rate-limited command) |

**Sienna-specific and unmeasured:** the TSS-P branch of the PID, `kiV = [3.6, 2.4, 1.5]` at 0/5/35 m/s (TSS2: 0.5/0.25),
tuned by comma on DSU-unplugged Corolla/RAV4H/ES years ago and not exercised upstream since 2025-12; `longitudinalActuatorDelay`
0.15 s default. **Plan:** measure the first sDSU drive (accel command vs `aEgo` for oscillation, set-speed step response, coast
fraction via `longitudinalPlan.allowThrottle`, stop/resume timing) and only then tune `kiV`, actuator delay, `ACCEL_MAX` or
personality. Do not tune blind.

## 5. How the measurements were made (reproducible)

- Logs: `ssh comma`, `/data/media/0/realdata/<route>--<seg>/rlog.zst`; copy with `tar` over ssh; read with
  `openpilot.tools.lib.logreader.LogReader([paths])`.
- Lane offset: `modelV2.laneLines[1].y[0]` (left inner), `[2].y[0]` (right inner), `laneLineProbs`; road edges `roadEdges[0/1]`.
- "Correction active" in a log: rolling 1 s median of `controlsState.desiredCurvature − modelV2.action.desiredCurvature`,
  |median| > 3e-5 (single-frame differences are clip noise).
- Sign of curvature: compare `sign(modelV2.action.desiredCurvature)` with the lane-center y at 40 m ahead, and with
  `carState.steeringAngleDeg`.
- Real controlsd over a log: `replay_process_with_name("controlsd", lr, custom_params={...})` from
  `openpilot/selfdrive/test/process_replay/process_replay.py`; on macOS the script needs an `if __name__ == "__main__":` guard.
- Calibration timeline: `extrinsicsCalibration.calStatus/rpyCalib/validBlocks` over time (recalibrating = mount-change detector).

## 6. Open items

- Lane offset correction: unmeasured with the correct sign; optional now that the car centers after the remount.
- sDSU: 0x2FF emission unverified; first drive pending.
- Longitudinal tuning for the Sienna: pending measurement.
- Other TSS-P cars with a stop timer: deliberately left on stock longitudinal (#3076 handling not restored).
- GitHub issues are disabled on both forks; findings live here and in commit messages.

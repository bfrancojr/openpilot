# alka-toyota fork: findings and decisions

Working notes for this fork of openpilot (bfrancojr/openpilot + bfrancojr/opendbc), written so that the reasoning behind the
code survives the chat sessions that produced it. Car: **Toyota Sienna 2018-20 (TSS-P, non-SecOC, fingerprint TOYOTA_SIENNA)**,
device: **comma 4 (mici)**. Dates are 2026. Commit hashes refer to the two forks.

## Branches

| Branch (both forks) | Purpose | State |
|---|---|---|
| `alka-toyota-long` | Always-on lateral, lane offset correction, driver monitoring settings, openpilot longitudinal on TSS-P (DSU unplugged or smartDSU). The device's branch; where new work lands. | openpilot `3ca849d1d`, opendbc `5ea42503` and later |
| `alka-toyota` | Kept identical to `alka-toyota-long` once that is validated on the car. | same as `alka-toyota-long` |

`alka-toyota` was frozen at openpilot `93bfcc38a` / opendbc `e385d87c` (lateral only) until 2026-09-29, when both forks' branches
were reset to `alka-toyota-long` after the owner had driven it for thousands of miles without problems. Sync by resetting
`alka-toyota` to `alka-toyota-long` (a fast-forward when nothing landed on `alka-toyota` directly), not by merging.

Install/switch: `installer.comma.ai/bfrancojr/<branch>`, or Settings → Software → target branch. The device builds on the
first boot after a change (no prebuilt marker); Params keys are compiled into the params library, so a new key needs that build.

## 1. Always-on lateral (steer whenever the cruise main switch is on)

Port of dragonpilot's ALKA idea onto upstream master (e3def3769, 2026-09-03). Decisions the owner made: the panda gates on the
stock ACC main switch (not a blanket flag); driver monitoring treats lateral-active as engaged (configurable since 2026-09-29,
see section 3); Toyota TSS-P/TSS2 non-SecOC only.

- Panda (opendbc safety): `ALT_EXP_ALWAYS_ON_LATERAL` (32) and `ALT_EXP_ALWAYS_ON_LATERAL_WHILE_BRAKING` (64);
  `get_lateral_allowed() = controls_allowed || (flag && acc_main_on && (!brake_pressed || while_braking))` in every steering check.
  Toyota reads `PCM_CRUISE_2.MAIN_ON` (0x1D3 bit 15, 33 Hz) only with safety param `ToyotaSafetyFlags.ACC_MAIN_ON` (16<<8).
  rx checksum/timeout failures clear `acc_main_on` like `controls_allowed`. Coverage 100 % on the touched safety files, MISRA clean.
- openpilot: `card.py` decides once at start from the `AlwaysOnLateral` / `AlwaysOnLateralWhileBraking` params and records the
  alternative-experience bits + safety param in CarParams, so panda and selfdrived cannot disagree. `selfdriveState.lateralActive`
  (capnp @14) = engaged-active OR (main on AND no NO_ENTRY/SOFT_DISABLE/IMMEDIATE_DISABLE event; `pedalPressed` exempt when
  steer-while-braking). controlsd steers on it, DM counts it as engaged (by default), UI shows a blue path (`UIStatus.LATERAL`).
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

## 3. Driver monitoring settings

**Complaint (2026-09-29):** attention alerts while looking at the road, with a hand resting on the armrest next to the phone the
owner uses as a GPS (the car has none), and then a lockout ("Too Distracted, N minutes left") that also stops always-on lateral.
Getting out of a lockout meant restarting the car while driving, which the owner judged more distracting and more dangerous
than the alert itself.

**Most likely cause, not confirmed from a log:** the DM model's phone detector (`phoneProb` > `_PHONE_THRESH` 0.5 flags
`distractedTypes.phone`), which cannot tell a mounted phone from one in the hand. To confirm, read
`driverMonitoringState.visionPolicyState.distractedTypes` (pose / eye / phone) around the alert in the route's rlog.

**How upstream escalates (`selfdrive/monitoring/policy.py`, `selfdrive/selfdrived/events.py`):** the red alert
(`driverDistracted3` / `driverUnresponsive3`) is a PERMANENT alert and does not disengage by itself. Red unanswered for 5 s sets
`noResponseForceDecel` (controlsd slows the car). A second red alert in a drive, or one unanswered red, starts the lockout:
1 / 5 / 15 / 30 min for successive lockouts, cleared on ignition. `tooDistracted` is NO_ENTRY, so it also blocks always-on
lateral (`always_on_lateral_allowed` treats any NO_ENTRY as blocking).

**Settings added (all live-reloaded by dmonitoringd every 2 s, no restart):**

| Param | Values | Effect |
|---|---|---|
| `DriverMonitoringMode` (int, default 2) | 0 off, 1 engaged only, 2 engaged + always-on lateral | 1 stops counting `lateralActive` as engaged. 0 skips alert and lockout logic, clears an active lockout, suppresses the uncertain-camera offroad alert; the DM icon is hidden |
| `DMIgnorePhone` (bool) | | phone no longer counts as a distraction; head pose and eye closure still do |
| `DMDisableLockout` (bool) | | alerts including red and the no-response slowdown still happen; no lockout starts and an active one is cleared. The red / no-response counts restart when they would have locked out, so turning the lockout back on does not lock out immediately |

`AlwaysOnDM` is unchanged and still applies in mode 1 (monitors while disengaged, never reaching red). Owner's intended setup:
`DMIgnorePhone` and `DMDisableLockout` on, mode 2 or 1. Upstream's note in `policy.py` warns that forks which disable or weaken
DM risk being banned from comma's servers.

## 4. Stop-and-go: openpilot longitudinal on a TSS-P car

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
  installed a BearTech C-SDSU on 2026-10-20; it emits 0x2FF like comma's smartDSU (measured below).
- "Cruise Fault, restart car" on first use = the PCM's low-speed lockout (`PCM_CRUISE_2.LOW_SPEED_LOCKOUT == 2`) not clearing.
  Community fix from 2020: switch the car fully off with the comma still powered, wait one to two minutes, start again.

**First-drive checklist (either hardware path):** CarParams in the rlog shows `openpilotLongitudinalControl`, `enableDsu` or
`flags & 2`; no `relayMalfunction` / `controlsMismatch` events; engages below 19 mph; holds a stop and resumes; then the
longitudinal measurements below.

**First smartDSU drives (2026-10-20, routes 0x10b 47 min and 0x10c 36 min; 0x10a earlier that day was still stock):**
- CarParams: `flags` 258 (`SMART_DSU` | `NO_STOP_TIMER`), `openpilotLongitudinalControl`, `autoResumeSng`, `enableDsu` false,
  DSU still answers the fw query. Checklist passed: no relay malfunction, controls mismatch or cruise fault events; 22 min
  engaged, 8 min of it below 19 mph; held stops of up to 167 s and resumed 0.1–0.3 s after the planner asked to move.
- 0x2FF: bus 0, 51 Hz, 4 bytes, exactly the RetroPilot `smart_dsu` firmware layout (button, ctrl mode, state|counter,
  Toyota checksum); checksum and counter good on all 254,725 frames. The DSU's own 0x343 reaches the car only in the first
  ~10 s after power-on, before openpilot starts sending.
- `STATE` = 5 (`FAULT_TIMEOUT`) whenever openpilot is in control. Not a fault: the firmware resets its DSU timer only while it
  forwards the DSU's ACC_CONTROL. 0 before openpilot sends, 3 (CAN error) for 13 frames at one power-on.
- **Distance button (fixed, opendbc bcfeb730):** on TSS-P it is wired to the DSU, so with the smartDSU openpilot can see it
  only in 0x2FF `FD_BUTTON`. The restore had left that out (upstream #31741, removed in #32777): 19 presses on these drives,
  no personality change. Now read with SMART_DSU; replay of both drives gives 19 press/release pairs, CAN valid throughout.
- Disengagements at highway speed with no pedal: `CRUISE_STATE` 8 → 0 with no fault bit and no cancel from openpilot (once
  `MAIN_ON` → 0). The cruise stalk is not on the bus, so this reads as the driver cancelling; unconfirmed.
- Tracking (PID active, no pedals): aEgo − command mean within ±0.04 m/s², rms 0.11–0.13 above 15 m/s, 0.21–0.31 below;
  aEgo std 0.13–0.18 at set speed. That says the car follows the command; it does not say the command is calm (next item).

**Owner complaints after those drives (2026-10-21) and what the logs say:**
- *Experimental mode was on* (100 % of engaged time; the model chose the speed 56–63 % of it). The owner had assumed the
  smartDSU needs it; it does not. Settings → Toggles → Experimental mode off gives the normal planner.
- *"Brakes very often, little shake":* 99 ACC brake applications while moving, 66 shorter than 1 s, starting at commands
  of only −0.1 to −0.4 m/s². At a steady 65 mph with no lead (0x10b ~1108–1122 s) the planner asks ≈ 0 but ACCEL_CMD
  cycles −0.75..+0.55 every ~1.5 s with speed within ±0.15 mph. CarController replay (matches the logged command, mean
  difference 0.011) puts 65 % of that command variance in the PID integral (std 0.11 vs planner 0.05). The TSS-P `kiV`
  predates the future-aEgo error (opendbc #1559), which halved TSS2's; halved the same way on branch `tssp-long-ki`
  (opendbc 983cfc7a), undriven. Falsifier: brake on/off cycles at steady cruise do not drop; cost to watch: hill sag.
- *"Pulsing pump at low speed, ABS?":* not ABS — `VSC1S07.FABS` never set on either drive. It is the brake actuator pump
  Toyota uses for ACC braking, cycling with the brake: 17–45 on/off cycles per minute while slowing below 27 mph. There the
  planner, not the integral, makes 83–96 % of the command swing, and in Experimental mode that is the model.
- *"Have to tap the gas after a light; bumpy launch":* every held stop ended with the driver's gas. In 7 of 11 the lead had
  pulled away 0.4–7.2 s before; the model was the plan source and said `shouldStop` for most of that wait. That is
  Experimental mode's behaviour, not the car port. Launches after the tap: the driver's push to ~1.5 m/s², then openpilot
  braking back to a lead still creeping at 3–4 mph 10–17 m ahead.
- Plan: drive with Experimental mode off on the current build first, then `tssp-long-ki`; compare brake cycles per minute
  at steady cruise and the stop/launch behaviour.

## 5. Longitudinal quality review (vs. the 2020 Reddit thread "2018 Sienna questions")

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

## 6. Steering: highway weave and sharp-curve limits

**Complaints (2026-09-30):** a subtle, slow weave on straight roads above ~50 mph, with the torque bar swinging left and right;
it predates this fork and happened on the owner's comma 2 too; the owner holds the wheel to damp it. And too little steering in
sharp curves even at low speed.

**Measured (2026-09-30, 825 qlog segments, routes 0x8e–0xbd, 228 min lateral-active; rlogs for the worst episodes):**

- **Stick-slip on centre.** The wheel stays still for ~1.2 s while `steeringTorqueEps` changes ~280–340 units, then jumps
  0.3–1.5°. The gyro (`deviceMotion.angularVelocityDevice.z`, r = 0.94 with angle × v) confirms that the car's yaw changes only
  when the wheel breaks free. This happens 11–17 times a minute at every speed. The lateral accel per jump grows with v²: p90
  0.09 m/s² below 34 mph, 0.26 at 45–56, 0.32 at 56–78 mph.
- **Not the driver's hands:** with |`steeringTorque`| < 6 (effectively hands-off, 5 min) the rate is the same, 12.3/min, and
  jumps are p90 0.20 m/s². A firm hold (30–100) cuts them to 0.07. The learned delay and friction were largely learned hands-on.
- **Why the controller lets it happen:** `get_friction` reaches its full ±friction·latAccelFactor (≈0.24 m/s²) only when
  |error + 0.3·jerk| > `FRICTION_THRESHOLD` 0.2. At 100 Hz on the highway that is 1–12 % of the time, so it acts as extra gain,
  not a breakaway push. The torque creeps up until the wheel breaks free and overshoots.
- **Car specifics:** `STEER_TORQUE_SENSOR.STEER_ANGLE` (0x260) is always 0 on this Sienna, so only the 0x25 angle (0.1°)
  is used. Learned: `lateralDelay` 0.45–0.47 s, friction 0.12–0.18 (fleet 0.14, typical Toyota), latAccelOffset
  −0.15 to −0.32, paramsd angle offset ≈ −4°.
- **Sharp curves are the torque ceiling.** In curves wanting > 1 m/s², the command is at the full 1500 units 91 % of the time at
  7–16 mph, 74 % at 16–25 and 64 % at 25–34. `clip_curvature` trimmed the model > 0.002 1/m in 11 / 2 / 0 % of those frames.
  Intersection turns also hit the 100°/s steer-rate torque cut. The model isn't the limit; 1500 is the panda/EPS limit.

**Steering friction boost (param `SteerFrictionBoost`, live-reloaded every second, torque-controlled cars only):** friction
compensation saturates at 0.1 m/s² (`FRICTION_THRESHOLD_BOOSTED`) instead of 0.2. Same magnitude, applied sooner. On the four
worst highway rlogs, full push goes from 1–12 % to 22–41 % of frames. The raw term can step 110–170 units in 10 ms, but the
car's torque rate limits (+15 / −25 per frame) bound reversals to ~0.2–0.3 s. Off is identical to upstream. Evaluate on a
straight with the hands resting lightly: a firm grip damps both the weave and the fix.

## 7. How the measurements were made (reproducible)

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
- Stick-slip: over lateral-active stretches, find runs where `carState.steeringAngleDeg` stays within 0.05° for ≥ 0.8 s. Record
  the `steeringTorqueEps` range during the run and the angle jump just after. Check against the gyro yaw rate × vEgo.
- Without the openpilot env: a venv with pycapnp, numpy and zstandard. `capnp.load("openpilot/cereal/log.capnp",
  imports=["openpilot/cereal", "opendbc_repo/opendbc/car"])` then `log.Event.read_multiple_bytes(zstd-decompressed rlog)`.

## 8. Open items

- Lane offset correction: unmeasured with the correct sign; optional now that the car centers after the remount.
- sDSU: verified on the first drives (section 4). Distance button fix undriven: check that a press changes the personality
  and the car's distance bars.
- Longitudinal tuning for the Sienna: first two drives show good tracking (section 4); revisit only on a complaint.
- Other TSS-P cars with a stop timer: deliberately left on stock longitudinal (#3076 handling not restored).
- Driver monitoring settings: unit tests in `test_monitoring.py` written but not yet run; the phone detector as the cause of
  the armrest alerts is unconfirmed until read from a route; settings screens not yet checked on the device.
- Steering friction boost: undriven. Measure stick events/min, jump size and `steeringTorqueEps` std on the same straights with
  it off and on, hands resting lightly; watch for torque chatter. If it helps but not enough, the next knobs are the threshold
  and an angle deadzone. The long learned delay (0.46 s) is also worth a hands-off re-learn.
- GitHub issues are disabled on both forks; findings live here and in commit messages.

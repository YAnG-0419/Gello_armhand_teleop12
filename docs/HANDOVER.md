# Repository handover

State as of the evening of 2026-07-26. Operator commands are in
[HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md); camera recovery is in
[ORBBEC_CAMERA.md](ORBBEC_CAMERA.md).

## Current system

- Primary arm input: PICO optical hand-root poses (`--input hand-roots`).
- Optional simultaneous hand input: PICO skeletons retargeted to both
  LinkerHand G20 hands (`--hands`).
- `docker compose up` starts arm control, hand control, Orbbec, the PICO ROS
  bridge, and the teleop safety gateway.
- Recording requires state and action for both FR3s and both G20 hands, plus
  RGB, 16-bit depth, and camera calibration.
- LeRobot export uses 54-dimensional state and action vectors: 14 FR3 joints
  followed by two 20-slot G20 vectors.

Hardware:

```text
left FR3    172.16.0.3
right FR3   172.16.0.2
host        enp6s0: 172.16.0.6/24, 192.168.1.53/24
Orbbec      192.168.1.10:8090
can0        left G20,  0x28
can1        right G20, 0x27
```

Robot-side control, updated 2026-07-26:

- Joint impedance gains are at the franka_ros2 example values
  (k 600/600/600/600/250/150/50, d 30/30/30/25/25/25/15). The previous halved
  set left a 20-40 mrad friction deadband on the distal joints, measured as
  stick-slip; the old values are kept in a comment in `controllers.yaml` for
  rollback.
- `JointImpedanceController` first-order-hold interpolates `q_goal` between
  incoming commands (ramp over the measured command spacing, clamped to
  1-20 ms, evaluated per 1 kHz cycle). This removes the k-gain-proportional
  torque step each 100 Hz command used to cause, and smooths the former snap
  from the move-to-start trajectory onto the live stream. Costs one command
  period (~10 ms) of target lag.
- `joint_state_broadcaster` and `franka_robot_state_broadcaster` run at
  200 Hz. They were 30 Hz, which aliased everything above ~15 Hz in host
  logs and hid slip transients.
- Host-side position EMA time constant is 0.20 s (raised from 0.10 after an
  offline tau sweep; see the jitter section).

## Hardware verification status

Verified:

- Dual-FR3 teleoperation from PICO hand roots.
- Simultaneous PICO-to-G20 hand teleoperation.
- Orbbec Viewer and ROS RGB-D streaming; camera traffic caused no packet loss
  in ping tests to either FR3.
- Stiffer gains + command interpolation + 200 Hz broadcasters, on recordings
  20260726_180408/182543/183228: measured/commanded speed variability at
  parity (0.61-0.65 vs 0.55, was ~1.1), proximal stick-slip roughly halved,
  no lunges, high-frequency arm gain 0.1-0.35 with no resonance. Operator
  reports clearly less jerky motion; the initial harshness after the gain
  change was resolved by the interpolation.
- `q` in the teleop keyboard flow exits cleanly.

Not yet hardware-validated end to end:

- recording a complete FR3/G20/RGB-D episode;
- LeRobot export followed by four-device action replay;
- per-side arm-gates-hand behavior and the `O`/`H` keyboard flows;
- the latest thumb constraint gating.

Robot replay and `H` reset cause physical motion. Keep PICO disengaged and
validate the path before using either.

Unresolved incident: on the first run after the 2026-07-26 evening restart,
the right FR3 made one violent motion at startup and hit a reflex stop; the
second run was normal. The container logs were lost to `docker compose down`
before they were read. Whenever a reflex trips, save
`docker compose logs franka-control` before taking the stack down, and treat
the first engagement after any restart as suspect until this is explained.

## EE jitter: state of the investigation

Analyzed recordings: `20260726_1712` (fixed and adaptive EMA baselines, soft
gains), `20260726_180408` (stiff gains + interpolation, arm only),
`20260726_182543` and `20260726_183228` (same, with `--hands`). Analyzer:
`teleop_sources/pico/scripts/simulation/analyze_ee_jitter_spectrum.py`
(wrap-safe: logged world-frame rotation vectors wrap near |r| ~ pi, so windows
are rebuilt from per-tick geodesic increments before spectral analysis;
`analyze_follow_log.py` keeps the aggregate following metrics).

Two mechanisms were separated:

1. Robot-side stick-slip and harshness - resolved by the gain/interpolation
   changes above. j5-j7 retain residual slip events (p95 error up to
   ~75 mrad in `--hands` sessions); revisit only if it stays visible in
   practice.
2. Quiet-band input noise - still open. Raw optical wrist rotation noise
   (quiet 0.5-3 Hz RMS) varied 7-31 mrad across the four sessions; the EMA
   only attenuates above ~3 Hz and the arm follows the remainder with gain
   0.6-0.8, so perceived quiet tremor tracks that session-to-session optical
   variation (hand position in headset view, pose, lighting), not code
   changes. Keeping the hand centered in the headset's view helps.

Dead ends, measured, do not repeat:

- Fixed-EMA sweeps: tau 0.10 -> 0.30 removes only ~35% of the quiet wander
  while tripling moving lag (lag scales with hand speed).
- The error-adaptive rotation EMA in `config/pico.yaml` failed on hardware
  because `rotation_error_low` (15 mrad) sits below the measured noise floor,
  so noise itself switches the filter fast. Its parameters are still active
  and harmless, but any retuning must put the low threshold above ~30 mrad.
- Averaging skeleton landmarks (rigid palm fit) does not reduce noise: the
  skeleton wanders as a whole, per-joint noise is coherent.
- Do not re-soften robot gains to hide the quiet band; that reintroduces
  stick-slip.

Remaining levers, in recommended order: speed-adaptive input filtering with
thresholds above the measured noise floor (One-Euro-style, displacement over a
~300 ms window, hysteresis), or tracker/skeleton fusion (tracker position is
sub-millimetre with optical fix but freezes in side-grasp poses, which is why
hand-roots became primary).

## Jitter instrumentation

```bash
RUN_DIR=/home/descfly/franka_teleop_data/diagnostics/$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --input hand-roots --hands \
  --debug-log "$RUN_DIR/ee_jitter.jsonl" \
  --hand-debug-log "$RUN_DIR/hand_fidelity.jsonl"
```

`FollowDebugLogger` records at 100 Hz: raw pre-EMA PICO wrist pose, filtered
pose, mapped target, commanded and measured joints, and FK end-effector poses
of both. `--hand-debug-log` adds canonical skeleton landmarks, emitted G20
joints, and thumb fidelity residuals per solved frame; summarize with
`scripts/simulation/analyze_hand_retarget_log.py`.

Useful facts:

- PICO skeletons update at ~52 Hz sample rate but deliver changed wrist
  samples on ~85% of 100 Hz ticks; the owner loop runs at 100 Hz.
- `PoseEma` advances only on changed skeleton samples. Position tau 0.20 s;
  rotation adapts 0.30 s -> 0.075 s between 15 and 80 mrad tracking error
  (thresholds known-flawed, see above).
- host and gateway `max_joint_speed` are both 0.5 rad/s; commanded joint
  speed rides that clamp at p95 during ordinary motion.
- arm commands pass through `teleop_interfaces/ArmCommand`; only the safety
  gateway publishes the FR3 command bus.
- hand retargeting runs after the arm command and at most one hand is solved
  per owner tick.

Keep the arm loop, camera load, and hand retargeting separable during tests:
measure first with `--hands` disabled, then enabled, on the same gesture.

## Current top priorities

1. Quiet-band input noise (see above): adaptive filtering or tracker fusion.
2. G20 hand behavior: hardware-validate the new fixed-root thumb mode (see
   Hand retargeting) and tune `THUMB_CMC_POWER_GRASP` for the operator's
   grips. Finger motion feeling slow was measured to be downstream of the
   software (input-to-emitted-qpos lag ~0 ms at 30 Hz solves): candidates are
   the G20 motors themselves, the 1500 unit/s bridge slew (~85 ms per
   half-swing; launch arg `max_command_rate`), and the 30 Hz send cap
   (`--hand-rate`, safe up to ~50). PICO thumb-tracking noise was ruled out
   as the fidelity limit (thumb landmarks jitter ~3 mm, same as the other
   fingers).

## Hand retargeting

Thumb mode reworked 2026-07-26 late evening, NOT yet hardware-validated: the
live pipeline runs the thumb in FIXED-OPPOSITION mode. Kinematic fact behind
it: at roll 0 the G20 thumb's yaw, pitch, MCP, and IP axes are parallel, so
(cmc yaw, cmc roll) set the direction of the thumb's curl plane while pitch
and the coupled MCP/IP flex curl within it. The mode locks
`THUMB_OPPOSITION_YAW_ROLL` (0.90/0.00 in `hand_retarget.py`) and drives
pitch + flex together across their FULL ranges from one normalized curl
signal: the operator's thumb bend mapped linearly over
`THUMB_CURL_BEND_RANGE` (0.25-1.30 rad, from the operator's measured usage).

History that led here, all measured offline (originals of 20260726_183228
were overwritten by a reused RUN_DIR; replay copies existed in session
scratch): mimicking the human thumb root is a conflicted objective on this
heterogeneous mechanism (a reachability oracle showed thumb-index tips can
touch exactly while the solver's own equilibrium left 16 mm of pinch gap;
freeing the flex actuator, 20x more iterations, and fading direction terms
all failed to close it). A first fixed-root attempt locked all three CMC
joints; the operator rejected it - the root must still bend, and flex alone
used only a third of its travel. The reworked mapping, replayed on the
operator's own rejected-session movements: yaw/roll exactly constant, pitch
and flex both sweep 100% of range, flex-to-bend correlation 0.997.

The opposition default was chosen offline (fingers half-curled around a
tool, curl sweep passes within 7 mm of the index/middle grasp line from
80 mm open); the operator already reported the previous offline-chosen pose
felt wrong, so expect to tune `THUMB_OPPOSITION_YAW_ROLL` (and possibly
`THUMB_CURL_BEND_RANGE`) on hardware with `inspect_thumb_configuration.py`.
Constructing `L20Retargeter` without `thumb_opposition_fixed` restores the
previous full solver (tests cover both).

- The public packet has 21 joint names; Pinocchio solves the 16 physical
  actuators and expands the five URDF mimic joints.
- Thumb MCP/IP flex is one coupled actuator and follows the robot FK bend
  curve; flex is fixed before solving the three CMC joints.
- Thumb segment-direction and local-frame constraints are adapted from the
  read-only `somehand` reference; weights fade in with flexion or fingertip
  proximity, and thumb-to-fingertip distance terms activate only near pinch.
- No ordinary teleop log is used as open/closed calibration ground truth.
- Gesture EMA alpha is 0.7. The bridge retains its 250 ms watchdog, 30 Hz
  cap, and 1500 vendor-unit/s slew limit.
- `inspect_thumb_configuration.py` visualizes or sends isolated thumb
  configurations; hardware mode does not send FR3 commands.

## Data pipeline

Required recording topics cover left/right measured FR3 joint state, executed
FR3 action, measured G20 state, post-mapping post-slew G20 action, RGB, depth,
and both camera-info topics. The raw rosbag is the synchronized source of
truth. Camera-only replay publishes under `/replay/camera`; robot replay is
separate and prepositions all four devices before sending actions.

## Safety and process invariants

- Do not run `RobotLinuxDemo` beside a Python XRoboToolkit client.
- `isActive` and array-change detection are required because the SDK can
  serve plausible cached skeletons after tracking loss.
- A hand-root fault on an engaged side disengages that arm; skeleton finger
  loss stops that hand but does not independently disengage an arm.
- One SDK client owns both arm and hand input.
- Do not run `teleop_hands.py` and `teleop_dual_fr3.py` together.
- Do not run OrbbecViewer while the ROS Orbbec service owns the camera.
- Do not reconfigure `enp6s0` during an active FCI session.
- The host operator must remain ROS-free; `env_guard.py` scrubs ROS
  variables.
- `config/pico.yaml` is validated by both the host parser and
  `pico_teleop_bridge/launch/pico.launch.py`.
- Another automation agent (a Cursor sandbox) has been observed inspecting
  this machine; if services change state unexpectedly, check whether someone
  else is operating it.

## Tests

```bash
cd /home/descfly/hsc/franka_upper_body_teleop

conda run --no-capture-output --name franka-teleop-pico \
  pytest -q teleop_sources/pico/tests

cd ros_ws/src/linker_hand_bridge
PYTHONPATH=. python3 -m pytest -q test/test_core.py
```

Data tests require the ROS Python environment:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools bash -lc \
  'source /opt/ros/humble/setup.bash &&
   export PYTHONPATH=/workspace/franka_upper_body_teleop/ros_ws/src/teleop_data:/workspace/franka_upper_body_teleop/ros_ws/src/teleop_core:${PYTHONPATH:-} &&
   python3 -m pytest -q /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/test'
```

Expected counts at handover:

```text
PICO host tests          87
LinkerHand bridge tests  30
teleop_data tests        15
```

The C++ controller rebuilds with
`docker compose run --rm tools bash /workspace/franka_upper_body_teleop/docker/build_workspace.sh --packages-select franka_fr3_arm_controllers`;
the workspace is volume-mounted with symlink-install, so config and Python
changes need no rebuild, only a service restart.

## Local data

```text
/home/descfly/franka_teleop_data/hand_coexistence.jsonl            PICO skeletons + tracker poses, 10 Hz
/home/descfly/franka_teleop_data/follow_debug.jsonl                degraded tracker trial (GUI contention)
/home/descfly/franka_teleop_data/diagnostics/20260726_1712/        fixed + adaptive EMA baselines, soft gains
/home/descfly/franka_teleop_data/diagnostics/20260726_180408/      stiff gains + interpolation, arm only
/home/descfly/franka_teleop_data/diagnostics/20260726_182543/      same with --hands (+ hand_fidelity)
/home/descfly/franka_teleop_data/diagnostics/20260726_183228/      same with --hands (+ hand_fidelity)
```

These are diagnostic recordings, not calibration ground truth.

# Repository handover

State as of 2026-07-26. Operator commands are in
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

## Hardware verification status

Verified:

- Dual-FR3 teleoperation from PICO hand roots.
- Simultaneous PICO-to-G20 hand teleoperation.
- Orbbec Viewer and ROS RGB-D streaming.
- Camera traffic did not cause packet loss in ping tests to either FR3.

Operator feedback on the latest thumb retargeting:

- thumb-root rotation improved;
- thumb extension is better after removing the log-derived flex calibration
  and fading heterogeneous orientation constraints out near extension;
- overall behavior is improved but not considered finished.

Not yet hardware-validated end to end:

- recording a complete FR3/G20/RGB-D episode;
- LeRobot export followed by four-device action replay;
- per-side arm-gates-hand behavior and the `O`/`H` keyboard flows;
- the latest thumb constraint gating after this handover update.

Robot replay and `H` reset cause physical motion. Keep PICO disengaged and
validate the path before using either.

## Next priority: FR3 end-effector jitter

Observed by the operator:

- the teleoperator's hand pose looks stable;
- the corresponding physical FR3 end effector appears to tremble.

No cause has been established. Do not tune smoothing or controller gains before
localizing the jitter.

Existing instrumentation:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --input hand-roots --hands \
  --debug-log /tmp/ee_jitter.jsonl
```

`FollowDebugLogger` records, at 100 Hz:

- filtered input pose;
- mapped target pose;
- commanded joint state;
- measured joint state;
- FK end-effector pose of the commanded joint state.

It does not record the raw pre-EMA PICO wrist pose or measured-robot FK.
Add those before drawing conclusions. Then compare:

1. raw wrist versus filtered wrist;
2. filtered wrist versus mapped target;
3. target versus commanded FK;
4. commanded joints versus measured joints;
5. measured-joint FK versus the observed physical motion.

Useful facts:

- PICO skeletons update at about 52 Hz; the owner loop runs at 100 Hz.
- `PoseEma` advances only on changed skeleton samples and uses a 0.10 s time
  constant.
- host and gateway `max_joint_speed` are both 0.5 rad/s.
- arm commands pass through `teleop_interfaces/ArmCommand`; only the safety
  gateway publishes the FR3 command bus.
- hand retargeting runs after the arm command and at most one hand is solved
  per owner tick.

Keep the arm loop, camera load, and hand retargeting separable during tests.
Measure first with `--hands` disabled, then enabled, using the same static-hand
trial.

## Hand retargeting

- The public packet has 21 joint names; Pinocchio solves the 16 physical
  actuators and expands the five URDF mimic joints.
- Thumb MCP/IP flex is one coupled actuator and follows the robot FK bend curve.
- Flex is fixed before solving the three CMC joints.
- Thumb segment-direction and local-frame constraints are adapted from the
  read-only `somehand` reference.
- Direction/frame weights fade in with flexion or fingertip proximity.
- Thumb-to-fingertip distance terms activate only near pinch.
- No ordinary teleop log is used as open/closed calibration ground truth.
- Gesture EMA alpha is 0.7. The bridge retains its 250 ms watchdog, 30 Hz cap,
  and 1500 vendor-unit/s slew limit.

The script below visualizes or sends isolated thumb configurations; hardware
mode does not send FR3 commands:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/inspect_thumb_configuration.py
```

## Data pipeline

Required recording topics cover:

- left/right measured FR3 joint state;
- left/right executed FR3 action;
- left/right measured G20 state;
- left/right post-mapping, post-slew G20 action;
- RGB, depth, and both camera-info topics.

The raw rosbag is the synchronized source of truth. Camera-only replay publishes
under `/replay/camera`. Robot replay is separate and prepositions all four
devices before sending actions.

## Safety and process invariants

- Do not run `RobotLinuxDemo` beside a Python XRoboToolkit client.
- `isActive` and array-change detection are required because the SDK can serve
  plausible cached skeletons after tracking loss.
- A hand-root fault on an engaged side disengages that arm.
- Skeleton finger loss stops that hand but does not independently disengage an
  arm.
- One SDK client owns both arm and hand input.
- Do not run `teleop_hands.py` and `teleop_dual_fr3.py` together.
- Do not run OrbbecViewer while the ROS Orbbec service owns the camera.
- Do not reconfigure `enp6s0` during an active FCI session.
- The host operator must remain ROS-free; `env_guard.py` scrubs ROS variables.
- `config/pico.yaml` is validated by both the host parser and
  `pico_teleop_bridge/launch/pico.launch.py`.

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
PICO host tests          83
LinkerHand bridge tests  30
teleop_data tests        15
```

## Local data

```text
/home/descfly/franka_teleop_data/hand_coexistence.jsonl
/home/descfly/franka_teleop_data/follow_debug.jsonl
```

These are diagnostic recordings, not calibration ground truth. The first
contains PICO skeletons and tracker poses; the second captures an earlier
degraded tracker-following trial.

# Repository handover

This document explains the current state of the repository for the next
developer or coding agent. `HARDWARE_DEPLOY.md` remains the canonical operator
procedure; do not turn this file into a second, divergent runbook.

## Current outcome

The mounted dual-FR3 workcell has working PICO teleoperation through either:

- two PICO controllers, using each grip as a hold-to-run clutch; or
- two wrist motion trackers, selected by serial number and activated from the
  keyboard.

The operator has tested motion-tracker control on the real setup and reported
smooth bimanual motion after the measured robot-base calibration was installed.
The controller route predates this work and remains supported.

Input selection is explicit and is not stored in YAML:

```text
--input controllers
--input motion-trackers
```

The main feature baseline is commit `de37e6b`:
`Fix motion tracker teleoperation for calibrated dual FR3`.

## Architecture and ownership

```text
PICO SDK
  -> controller/tracker adapter
  -> shared XR-to-world mapping
  -> per-arm relative pose clutch
  -> bimanual Pink/Pinocchio IK
  -> validated UDP command/state protocol
  -> pico_teleop_bridge
  -> /teleop/arm_commands (ArmCommand)
  -> safety gateway
  -> /target_robot/joint_commands
  -> per-arm joint splitter
  -> FR3 impedance controllers
```

Robot state travels in the opposite direction through the ROS bridge and UDP
backend. Replay enters at `/teleop/arm_commands`, so live PICO and replay share
the same ROS safety gateway.

Important ownership rules:

- `teleop_core.contract` owns robot topics and canonical joint names.
- Every source publishes `teleop_interfaces/ArmCommand`.
- Only `teleop_core.safety_gateway` may publish the shared FR3 command bus.
- PICO device I/O and host IK stay under `teleop_sources/pico`.
- Robot control and data tooling must not acquire a PICO dependency.
- Existing sibling repositories are references only; do not modify them.

Key files:

- `config/pico.yaml`: both PICO input configurations, pose scales, IK rate, and
  UDP endpoints.
- `teleop_sources/pico/src/pico_bimanual_franka_teleop/xr_input.py`: controller
  and tracker acquisition, activation, and tracker fault checks.
- `teleop_sources/pico/src/pico_bimanual_franka_teleop/pose_mapping.py`:
  coordinate conversion and relative pose clutch.
- `teleop_sources/pico/src/pico_bimanual_franka_teleop/ik.py`: dual-arm IK.
- `ros_ws/src/pico_teleop_bridge`: UDP/ROS boundary.
- `ros_ws/src/teleop_core`: source arbitration and final command safety.
- `ros_ws/src/franka_fr3_arm_controllers`: hardware controllers and reset.
- `ros_ws/src/teleop_data`: recording, conversion, replay, and operator CLI.

YAML parsing is intentionally strict: missing and unknown fields are errors.

## Coordinate and workcell model

PICO/OpenXR axes are interpreted as:

```text
+X right, +Y up, -Z forward
```

Robot world axes are:

```text
+X forward, +Y left, +Z up
```

Therefore the shared mapping is:

```text
PICO +X -> robot -Y
PICO +Y -> robot +Z
PICO -Z -> robot +X
```

Both controller and motion-tracker inputs use this same mapping. Relative pose
control anchors the input pose and current link-7 pose when a side is engaged;
releasing that side clears its anchor. Re-engagement starts from the current
robot pose instead of jumping back to an old absolute target.

The current dual-arm geometry is the measured remounted workcell, not the
legacy `dual_arm_45` arrangement:

```text
left base:
  xyz =  0.013079894,  0.100675805, -0.005974552
  rpy = -0.78850572,   0.02334617,   0.01037711

right base:
  xyz = -0.013079894, -0.100675805,  0.005974552
  rpy =  0.76133216,   0.03792674,  -0.01038175
```

Keep `dual_fr3_kinematics.urdf` and `scene.xml` synchronized. Tests compare the
Pinocchio and MuJoCo base frames and ensure the MuJoCo home keyframe matches
the captured hardware reset pose.

## Motion-tracker specifics

The configured wrist trackers are:

```text
left:  PC2310MLL6020917G
right: PC2310MLL5290914G
```

The PICO app must use Object/Motion Tracking mode, and Motion Tracker Mode must
be `Object`, not `None`. Other connected trackers are accepted and ignored;
the two wrist inputs are selected by serial, not array order.

XRoboToolkit client ownership is important:

- keep `RoboticsService`/PC Service and the headset app running;
- close the desktop `RobotLinuxDemo` GUI before Python connects;
- never run tracker inspection and teleoperation at the same time;
- do not suppress native SDK diagnostics.

The desktop GUI and Python SDK empirically compete for the feedback stream.
The SDK/PC Service has not been proven to support multiple simultaneous
consumers safely. `inspect_motion_trackers.py` is consequently a separate,
short-lived SDK client and refuses to start when it detects the desktop GUI.

The vendored SDK binding was extended to support more than three motion
trackers, use the top-level frame timestamp when Motion has none, expose
consistent tracker arrays, clear stale tracker caches on initialization, and
put native diagnostics on readable line boundaries. Re-run
`scripts/setup_pico_env.sh` after changing this binding.

`tracker_to_control` is still identity for both wrists. If tracker rotation
causes unwanted translation, calibrate the rigid tracker-to-control-point
transform rather than changing the global XR axis mapping.

## Activation and safety behavior

Controller input starts blocked. After startup or stale data, release a grip
once before acquiring that arm again.

Motion-tracker keyboard controls are:

- `Space`: toggle both sides
- `L` / `R`: toggle one side
- `X`: disengage both sides
- `Q`: disengage both sides and exit

An inactive tracker may move freely. Its pose continues updating the safety
baseline but does not move its robot arm. The first frame after re-enabling a
side reanchors control, allowing the operator to relax or reposition an
inactive hand without creating a jump.

Tracker-side checks disengage on:

- missing, inconsistent, or stale stream data;
- a frozen active pose;
- backward timestamps;
- excessive position or rotation jumps; or
- excessive linear or angular speed.

A restarted tracker timestamp disengages the arms but can recover on a new
stable stream and explicit re-engagement; it does not remain permanently
latched to the old timestamp.

Independent ROS-side checks then enforce:

- fresh measured state from both FR3 arms;
- allowed source and exclusive source/session ownership;
- monotonic command sequence numbers;
- exact joint schema and canonical ordering;
- finite values and FR3 joint limits;
- a small first-target delta from measured state; and
- per-cycle joint slew limits.

There is no dry-run service. Validated commands are sent to hardware whenever
the real control stack is running, but all PICO inputs start disengaged or
require explicit acquisition. Keep the emergency stop reachable.

## Reset behavior

`franka-control` starts `/reset_to_initial_pose`. Reset suppresses the normal
teleop gateway and directly sends a synchronized joint-space smootherstep
trajectory to both arm controllers.

The duration is computed from the largest displacement among all 14 joints:

- peak joint speed: `0.20 rad/s`
- peak joint acceleration: `0.40 rad/s²`
- minimum nonzero duration: `1.0 s`

A 1 rad maximum displacement takes about 9.4 seconds and 2 rad takes about
18.8 seconds. Reset aborts if either measured arm state becomes stale and only
reports success after the final error remains within tolerance.

Reset is not collision planning. The saved target and the interpolated path
must both be operator-validated. Disengage PICO before `/reset`.

## Known limitations

- The MuJoCo stand is only a proxy. The arm base frames are measured, but the
  surrounding collision geometry is not a calibrated digital twin.
- Tracker-to-control mounting transforms have not yet been calibrated.
- Tracker discovery cannot safely share the SDK connection with the GUI or
  live teleoperation.
- The tracker thresholds in `config/pico.yaml` are operational guards, not a
  formal human-robot safety certification.
- IK controls the FR3 link-7 frames. There is currently no robot hand, gripper,
  tactile, or finger-command contract.
- Reset uses joint interpolation without obstacle avoidance.
- Real hardware validation remains an operator task; automated tests are
  offline and cannot prove a collision-free workcell.

## Likely next work: hands or grippers

Clarify the hardware and intent before writing code. “Hand” could mean PICO
optical hand tracking as an input, a simple parallel gripper on each FR3, or a
multi-joint dexterous robot hand. These require different interfaces and
safety limits.

Preserve the working arm-pose path. Do not replace wrist motion trackers with
PICO optical hand poses merely to control fingers. Add hand/gripper commands
as a separate, synchronized channel so arm clutching, arm safety, recording,
and replay continue to work unchanged.

Before implementation, obtain:

- hand/gripper model and driver;
- per-side ROS command and state topics;
- joint names, limits, velocity/effort limits, and control mode;
- desired operator input: buttons, analog trigger, gestures, or glove;
- behavior when an arm is inactive or tracking becomes stale;
- required tactile/force feedback and fault behavior; and
- recording and replay schema requirements.

A suitable implementation should then:

1. define a device-independent hand/gripper command contract;
2. add source-specific input mapping under `teleop_sources/`;
3. add a dedicated safety gateway for hand commands;
4. extend hardware adapters without giving sources direct hardware access;
5. add hand state and validated commands to recording/replay;
6. extend MuJoCo assets and mock input; and
7. test per-side disengagement, stale data, limits, and arm/hand synchronization.

Avoid silently appending finger joints to `ArmCommand`: its current arm-only
joint schema and safety logic deliberately describe only the two FR3 arms.

## Verification before the next hardware session

Offline PICO tests:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  pytest -q teleop_sources/pico/tests
```

Mock MuJoCo:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/teleop_dual_fr3_mujoco.py \
  --config config/pico.yaml --input mock --headless --duration 2
```

ROS build:

```bash
./scripts/build.sh \
  --packages-select franka_fr3_arm_controllers pico_teleop_bridge teleop_core
```

Compose validation:

```bash
cd docker
docker compose config --quiet
```

Before real hardware, follow `HARDWARE_DEPLOY.md` exactly and first verify the
selected real PICO input in MuJoCo. Do not run the tracker inspector against a
live GUI or teleop SDK client.

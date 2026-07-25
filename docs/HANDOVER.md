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

The next concrete target is PICO 4 Ultra optical hand tracking for two Linker
Hands:

```text
PICO 26-joint hand skeleton
  -> retargeting method to be selected after the Linker model is known
  -> left/right Linker Hand commands
```

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

## PICO hand-tracking API

The required PICO 4 Ultra hand skeleton already reaches the vendored
XRoboToolkit binding. Do not add another SDK client. Extend the existing
controller or motion-tracker input object so arm poses and hand skeletons are
read through the same `xrt.init()` connection.

Current Python functions:

```python
left_joints = xrt.get_left_hand_tracking_state()
right_joints = xrt.get_right_hand_tracking_state()
left_active = xrt.get_left_hand_is_active()
right_active = xrt.get_right_hand_is_active()
frame_timestamp_ns = xrt.get_time_stamp_ns()
```

Each joint array currently has shape `(26, 7)`. Each row is:

```text
[x, y, z, qx, qy, qz, qw]
```

The upstream example describes `isActive` as `0 = low quality` and `1 = high
quality`. Treat any value other than `1` as invalid until this has been checked
against live PICO output.

The 26 rows follow the OpenXR `XrHandJointEXT` order:

| Index | Joint |
|---:|---|
| 0 | palm |
| 1 | wrist |
| 2–5 | thumb metacarpal, proximal, distal, tip |
| 6–10 | index metacarpal, proximal, intermediate, distal, tip |
| 11–15 | middle metacarpal, proximal, intermediate, distal, tip |
| 16–20 | ring metacarpal, proximal, intermediate, distal, tip |
| 21–25 | little metacarpal, proximal, intermediate, distal, tip |

The official Unity sender calls
`PXR_HandTracking.GetJointLocations(HandType.HandLeft/HandRight, ...)` and
serializes this per hand:

```text
isActive
count
scale
HandJointLocations[i].p   pose: x,y,z,qx,qy,qz,qw
HandJointLocations[i].s   OpenXR/PICO location-status flags
HandJointLocations[i].r   joint radius
```

It attaches a top-level `timeStampNs` to the complete XR frame. There is no
separate hand timestamp in the current PC binding. The existing
controller/tracker code reads the timestamp before and after its data getters
to reject a callback occurring between reads, but the hand API does not yet
provide one atomic frame getter.

Do not assume the global coordinate frame of these joint poses merely from
their field layout. Confirm it once against live data (wrist translation,
left/right hand identity, and a simple finger flexion). The correct Linker
mapping cannot be selected from the array shape alone.

Current binding limitations:

- it exports pose arrays and `isActive`, but not `count`, hand `scale`,
  per-joint status, or radius;
- it parses `scale` internally, but the C++ getter has the wrong integer return
  type and is not registered with pybind;
- it does not clear a hand array when a new frame contains fewer joints;
- it can retain the previous side when the Unity sender fails to obtain a new
  hand sample; and
- the binding cannot currently distinguish a valid stationary hand from a
  stale cached hand using hand data alone.

Before commanding hardware, the acquisition layer must make it possible to
tell a complete current frame from a partial or cached one. The API needs to
retain timestamp, active flag, count, pose, and the location-status fields
needed for that decision. This is an acceptance requirement, not a choice of
retargeting algorithm.

The quaternion bone-axis convention, usefulness of `scale`, and interpretation
of the raw location-status values still require verification. They are
deliberately not assumed here.

## Verified live PICO hand-tracking measurements

Measured on the real headset with
`teleop_sources/pico/scripts/hardware/inspect_hand_tracking.py`, a read-only
diagnostic that observes one SDK client and sends nothing to any robot. A 45 s
session with both wrist trackers worn in Object mode and hand tracking enabled:

- Hand tracking and Object Motion Tracking **do coexist** in one SDK client.
  Motion timestamp advance was statistically identical whether hands were
  active (median 97.8 ms, p95 111.8 ms) or inactive (median 97.8 ms, p95
  111.9 ms), and motion never went stale while a hand was live. Conditioned on
  hand availability, both signals were simultaneously fresh 100% of the time.
- Rates: XR frame and motion tracking about 69 Hz; optical hand tracking about
  52 Hz. The lower hand rate is the native optical rate, not contention.
- Judge coexistence only over windows where `isActive == 1`. Raw wall-clock
  time is misleading because hands leaving the headset cameras looks identical
  to contention. In the measured session every not-both-live second was the
  operator acquiring tracking before `t = 12 s`.
- The joint poses and the motion tracker poses share one global XR frame whose
  origin sits near head height, not on the floor: wrist `y` stayed within
  `[-0.64, -0.14]` m. A floor origin would put hands near `y = +1.0`.
- `isActive` is load-bearing. Complete, plausible pose arrays are still served
  when tracking is lost: fully valid poses appeared in 434 of 440 logged
  samples while `isActive == 1` in only 321. Never infer liveness from the
  array contents.
- No partial frames were observed. When `isActive == 1`, all 26 joints were
  populated.
- A stationary hand still jitters: zero consecutive active samples were
  bitwise identical. Change detection is therefore a usable liveness proxy,
  and a truly frozen cache would show as constant data.
- Left and right skeletons are genuinely mirrored, so PICO does not need the
  MANO x-flip. Reflection-signed volume of the wrist-to-metacarpal vectors was
  consistently negative for the left hand and positive for the right.

The skeleton is rigid and scaled per operator; inter-joint distances varied by
under 0.1 mm within a session. Measured landmark distances from the wrist:

```text
middle_metacarpal   0.0226 m
index_metacarpal    0.0285 m
palm                0.0522 m
middle_proximal     0.0819 m
index_proximal      0.0858 m
```

This is a retargeting trap worth stating explicitly. MANO index 9 is the
middle-finger MCP **knuckle**, but OpenXR `*_metacarpal` sits at the **base**
of the metacarpal bone near the carpus. The OpenXR analogue of the MANO
knuckle is `*_proximal`. Deriving a human hand scale from `*_metacarpal` makes
it 3.6 times too small, which inflates the robot-to-human scale ratio by the
same factor and saturates every finger joint.

Authoritative references used for this section:

- [PICO hand tracking documentation](https://developer.picoxr.com/home-api/document/unity/hand-tracking/)
- [OpenXR 26-joint enumeration](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrHandJointEXT.html)
- [XRoboToolkit Unity sender at the inspected revision](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/blob/c9326092ff4d11e8b507b041713194b93470a8e1/Assets/Scripts/TrackingData.cs)
- [XRoboToolkit Python binding at the inspected revision](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/c64ccf6acd577a333e03b66fafe8efeeceb511b1/bindings/py_bindings.cpp)

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
- IK controls the FR3 link-7 frames. PICO hand data is available at the native
  binding, but there is currently no Linker Hand integration in this repo.
- Reset uses joint interpolation without obstacle avoidance.
- Real hardware validation remains an operator task; automated tests are
  offline and cannot prove a collision-free workcell.

## Next work: PICO skeleton to Linker Hand

The intended input is now known: PICO 4 Ultra optical hand tracking. Continue
using the wrist motion trackers for FR3 link-7 pose control; use the optical
26-joint skeleton for Linker Hand finger control. These are separate signals:
loss of the optical skeleton must not be mistaken for loss of a still-valid
wrist tracker.

The exact Linker Hand model installed on each FR3 still needs to be recorded.
This is essential because L7, L10, L20, L21, and L25 have different command
dimensions and semantics.

The inspected official Linker Hand ROS 2 SDK uses:

```text
command: /cb_left_hand_control_cmd
         /cb_right_hand_control_cmd
state:   /cb_left_hand_state
         /cb_right_hand_state
type:    sensor_msgs/msg/JointState
```

Its standard `*_control_cmd` position values are model-specific hardware
ranges (official examples commonly use `0..255`), despite the `JointState`
message type. Do not assume radians. Older `*_control_cmd_arc` adapters expose
radian-oriented topics for some models, but the exact deployed SDK/model must
be confirmed before choosing an interface.

Official command ordering:

| Model | Command order |
|---|---|
| L7 | thumb flexion, thumb abduction, index/middle/ring/little flexion, thumb rotation |
| L10 | thumb base, thumb abduction, index/middle/ring/little base, index/ring/little abduction, thumb rotation |
| L20 | five bases, five abductions, thumb opposition, four reserved, five tips |
| L21 | five bases, five abductions, thumb roll, four reserved, thumb middle, four reserved, five tips |
| L25 | five bases, five abductions, thumb roll, four reserved, five middle, five tips |

The command-order table is transcribed from the inspected SDK README; confirm
it against the exact installed model and driver revision before sending a
command.

Confirmed requirements for the future work:

- acquire hand data through the SDK client already owned by teleoperation,
  never a second simultaneous `xrt.init()` client;
- verify live skeleton output, coordinate semantics, status behavior, and
  simultaneous Object Motion Tracking plus Hand Tracking;
- record the exact left/right Linker model and its installed driver interface;
- retarget the PICO skeleton to that model; and
- reject stale/missing input and prevent an excessive target delta or command
  speed. Reacquisition must not cause a command jump.

The retargeting representation and algorithm, internal software interfaces,
simulation scope, and recording format have not been selected. Do not infer
them from this handover or adopt a third-party retargeting library without
first validating it against the actual Linker model and the live PICO data.

Linker references inspected for this handover:

- [official Linker Hand ROS 2 SDK](https://github.com/linker-bot/linkerhand-ros2-sdk)
- [inspected Linker ROS 2 SDK revision](https://github.com/linker-bot/linkerhand-ros2-sdk/tree/7dea77ee947d5e91fa072a80e31ee777125c8003)

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

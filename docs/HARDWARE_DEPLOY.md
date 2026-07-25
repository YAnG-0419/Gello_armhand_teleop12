# Dual-FR3 hardware deployment

Run the setup in the repository README first. Keep the emergency stop
reachable.

Current workcell:

- left FR3: `172.16.0.3`
- right FR3: `172.16.0.2`
- host: `enp6s0`, `172.16.0.6/24`
- ROS domain: `0`

The combined Pinocchio and MuJoCo models use the measured world-to-base
transforms for the remounted workcell, not the legacy `dual_arm_45` CAD
transforms. The MuJoCo `home` keyframe matches the measured joint pose in
`ros_ws/src/franka_fr3_arm_controllers/config/initial_pose.yaml`. Hardware
`/reset` uses that separately captured joint-space target; the calibrated base
transforms do not alter its joint angles or prove that every interpolation path
to it is collision-free.

Before every session, activate both arms and FCI in Franka Desk:

```bash
ping -c 2 172.16.0.3
ping -c 2 172.16.0.2
```

## PICO input

The input is a required CLI argument.

Keep the XRoboToolkit `RoboticsService` and the app on the headset running, but
close the desktop `RobotLinuxDemo` GUI before starting Python teleop. The GUI
and Python SDK compete for the PC Service feedback stream; whichever connects
last can leave the other client open but no longer receiving fresh poses.

Controller mode:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --input controllers
```

Each grip controls one arm. After startup or stale data, release the grip once
before reacquiring that arm.

For motion-tracker mode, select object/motion tracking (not full-body tracking)
in the PICO XRoboToolkit app, confirm that both trackers are shown, and enable
data sending. Put the left and right tracker serial numbers in
`input.motion_trackers.serials` in `config/pico.yaml`.
Other connected trackers are allowed and ignored. Label the two selected wrist
trackers and always wear each serial on its configured side.

To inspect the available IDs before editing the configuration, first close the
desktop `RobotLinuxDemo` GUI and run this single SDK client:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/inspect_motion_trackers.py
```

It listens for ten seconds, prints every detected serial number, then closes
the SDK. It refuses to start while the competing desktop GUI is running.

The mounting transforms initially use identity. Calibrate them if tracker
rotation creates unwanted control-point translation.

Tracker input automatically disengages both arms on missing or stale data,
frozen poses, timestamp errors, pose jumps, or excessive linear/angular speed.
Re-engage explicitly after tracking is stable.

Then run:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --input motion-trackers
```

Tracker keyboard controls:

- `Space`: toggle both arms
- `L` / `R`: toggle one arm
- `X`: disable both arms
- `Q`: disable both arms and exit

Test either real input in MuJoCo by using the same `--input` value:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/teleop_dual_fr3_mujoco.py \
  --config config/pico.yaml --input controllers
```

Confirm forward/right/up maps to robot `+X/-Y/+Z`.

## Start the robot pipeline

Start XRoboToolkit PC Service and the selected PICO stream.

Terminal 1:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up franka-control
```

Terminal 2:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up teleop-control pico-bridge
```

Terminal 3 runs one of the PICO commands above. Input starts disengaged or
requires acquisition, so selecting a CLI mode does not immediately move an
arm.

## Operator

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools ros2 run teleop_data operator \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml \
  --qos /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording_qos.yaml
```

Commands include `/capture`, `/reset`, `/record`, `/stop`, `/save`, `/discard`,
and `/status`. Disengage PICO before `/reset`, and use only an operator-validated
captured pose and path.

`/reset` moves both arms together along a synchronized joint-space
smootherstep trajectory. Its duration is chosen from the largest of the 14
joint displacements, with peak limits of `0.20 rad/s` and `0.40 rad/s²` and a
minimum duration of one second. For example, a largest displacement of 1 rad
takes about 9.4 seconds; 2 rad takes about 18.8 seconds. The reset aborts if
either arm stops publishing fresh joint states. This is not collision
planning, so verify the path and keep the emergency stop reachable.

With `franka-control` already running, reset can also be invoked without the
operator shell:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools \
  ros2 service call /reset_to_initial_pose std_srvs/srv/Trigger '{}'
```

## Linker Hand G20 bring-up

Two LinkerHand G20 hands are installed. The bus-to-side mapping is confirmed
from each hand's own reported comm ID, not assumed:

```text
can0  comm id 0x28  LEFT   serial LHT20-010-502-L-B-1-D  embedded 1.0.9
can1  comm id 0x27  RIGHT  serial LHT20-010-556-R-B-1-D  embedded 1.0.10
```

The two hands run different embedded firmware, 1.0.9 on the left and 1.0.10 on
the right. Do not assume the sides behave identically. Both report hardware
version 4.0.1, structure version 10.1.0, and touch sensor type 2, which is not
the full-palm matrix type, so `is_touch` stays false.

The USB-CAN adapters have no USB serial number, so `can0` and `can1` are
assigned by USB port path: `7-1.1` becomes `can0` and `7-1.3` becomes `can1`.
There is no udev rule pinning this. Re-verify the mapping with the discovery
frame below after any re-cabling.

Bring both buses up at 1 Mbit/s. Other bitrates get no reply at all:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

If the hands are silent, unplug and reconnect the USB-CAN adapters before
debugging anything else. Both hands were completely mute until the adapters
were re-enumerated: the vendor driver crashed in its constructor and the
vendor's own discovery frame went unanswered. Note that a silent bus does not
raise CAN error counters here, because `pcan_usb` reports transmit completion
without bus acknowledgement and berr reporting is off, so clean counters are
not evidence that a hand is present.

To check presence without starting any driver, send the vendor's broadcast
discovery frame with the listener started first, then read the ASCII serial
from the multi-frame reply:

```bash
candump -t d can0 & sleep 0.4; cansend can0 0FF#C0; sleep 1; kill %1
```

The vendor driver is vendored into this repository at
`host_ws/src/linker_hand_ros2_sdk`, copied from the official SDK at revision
`7dea77ee947d5e91fa072a80e31ee777125c8003`. Only the driver package is
vendored; the GUI and pressure-diagram packages would add a PyQt dependency and
are not used. Nothing at runtime depends on the sibling `linkerhand-ros2-sdk`
checkout. Build it together with the bridge:

```bash
./scripts/build_host.sh
source host_ws/install/setup.bash
```

`--symlink-install`, which that script passes, is required. The vendored package
does not install its `LinkerHand/config/*.yaml` into `share/`, so a copying
install cannot find `setting.yaml` at runtime.

Start the driver per side. Use `linker_hand_sdk` with `hand_joint:=G20`, which
is motionless at startup:

```bash
ros2 run linker_hand_ros2_sdk linker_hand_sdk --ros-args \
  -p hand_type:=left -p hand_joint:=G20 -p can:=can0 -p is_touch:=false
```

Prefer it over `linker_hand_advanced_g20`, which commands
`DEFAULT_POSITION` at `DEFAULT_SPEED`/`DEFAULT_TORQUE` of 255 during
construction and therefore snaps the hand to a pose at full speed and torque
before any command is sent.

`linker_hand_sdk` has no G20 branch in its startup-pose code, so it never
initializes speed or torque for G20. The hand keeps whatever the firmware last
held, possibly 255. Set a conservative speed before the first position command:

```bash
ros2 topic pub --once /cb_hand_setting_cmd std_msgs/msg/String \
  '{data: "{\"setting_cmd\": \"set_speed\", \"params\": {\"hand_type\": \"left\", \"speed\": [30,30,30,30,30]}}"}'
```

Operating notes verified on this hardware:

- commands are `sensor_msgs/msg/JointState` on `/cb_{side}_hand_control_cmd`
  with exactly 20 values in the vendor `0..255` range; only `position` sets the
  pose, `velocity` acts as a per-joint speed limit and is ignored when all
  zero, and `effort` and `name` are ignored;
- the `_arc` radian topics have no subscriber anywhere in the SDK, so 0..255 is
  the only interface;
- publish at 30 to 60 Hz. Above about 100 Hz commands are silently dropped,
  because the driver's `list_check` sees `array.array` rather than `list` and
  refuses a new pose while one is pending;
- `/cb_{side}_hand_state` is only polled from hardware while something is
  subscribed, and its **first message carries 10 values, not 20**. Validate the
  length and discard early messages;
- reserved slots 11 to 14 read back 0 and should be commanded as 0;
- lower values mean more flexion; near 255 is open;
- `get_faults` returned all zeros on both hands, and its result prints in the
  driver's console rather than on a topic.

Verified end to end on the left hand: `set_speed` accepted, no faults,
re-commanding the measured position produced at most 1 unit of drift, and a
single-slot nudge of -30 tracked exactly with no other slot moving more than 4
units. Keep the arm powered down with FCI disabled during hand-only testing,
and confirm finger clearance to the arm and table before commanding motion.

## PICO hand tracking to Linker Hand

```text
PICO optical 26-joint skeleton
  -> canonical 21 landmarks
  -> L20 URDF retargeting on the host, in the franka-teleop-pico Conda env
  -> hand qpos datagrams on udp://127.0.0.1:5570
  -> linker_hand_bridge, 0..255 projection, watchdog and slew limit
  -> /cb_{side}_hand_control_cmd
  -> vendor linker_hand_sdk
  -> CAN
```

The whole hand path runs **on the host**, not in the container. The container is
ROS 2 Humble while the host is Jazzy, and the vendor driver has to be host-side
because it needs CAN. Cross-distro DDS between Humble and Jazzy is not a
supported guarantee, so the hand stream crosses that boundary as UDP rather than
as ROS topics. The arm stack in the container is unaffected, and losing the
optical skeleton is an independent event from losing a wrist tracker.

Build with `./scripts/build_host.sh`. `scripts/build.sh` builds the container
workspace and does not build the hand stack at all.

Bimanual hand teleoperation takes **two terminals**. Prerequisites: both CAN
buses up at 1 Mbit/s, both hands powered, the headset app streaming with hand
tracking enabled, XRoboToolkit PC Service running, and the desktop
`RobotLinuxDemo` GUI closed.

Terminal 1, the whole robot side. This starts both vendored drivers, renamed so
they do not collide, and the bridge, which requests a conservative joint speed
a couple of seconds later:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
source /opt/ros/jazzy/setup.bash
source host_ws/install/setup.bash
ros2 launch linker_hand_bridge hands.launch.py
```

Terminal 2, the operator side, in the Conda environment:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_hands.py
```

It prints `left: tracking, sending` per side once `isActive` is 1, and the reason
whenever a side stops. Stop it first when finishing; the bridge's 250 ms watchdog
then stops publishing and each hand holds position, with no return-home motion.

Useful arguments: `enabled:=false` maps and publishes only to
`/linker_hand_bridge/{side}/mapped_command` without commanding the hands,
`sides:=left` runs one hand, `max_command_rate:=` changes the slew limit, and
`initial_speed:=0` skips the speed request.

`teleop_hands.py` owns the single SDK client, so it cannot run at the same time
as `teleop_dual_fr3.py`. Hand teleoperation and arm teleoperation cannot yet
share a session.

To replay a recording instead of a live headset, substitute terminal 2 with
`scripts/simulation/replay_hand_log.py --log <file> --port 5570 --rate 30`.

`enabled` is read once at startup and cannot be turned on through a live
parameter update. A fresh valid datagram is required before anything is
published, output is capped at 30 Hz, motion is slew-limited, and a 250 ms input
timeout stops publication. On reacquisition the limiter re-anchors on measured
hand state when available so there is no command jump. There is no automatic
return-home.

### Per-finger length normalization

The L20's four non-thumb fingers are all the same length, 0.0998 m, while a
human's differ substantially. Measured against one operator after palm-width
scaling, the robot's pinky ran about 34 mm long, the index 15 mm, the ring 11 mm,
and only the middle finger matched.

Matching absolute tip positions therefore made every over-long finger curl far
harder than the operator did. The pinky tip command averaged 70 of 255, deeply
flexed, when the operator's pinky was not. Long fingers curled deeply enough to
cross into their neighbours, which is what fouling on hardware looked like.

`L20Retargeter` now rescales each finger's target chain so its length equals the
robot's own finger, preserving every segment direction and so the exact curl
shape the operator made. Disable it with `normalize_finger_length=False` to
compare. Measured effect on real recorded frames:

```text
                           before   after
IK loss, right hand       0.01005  0.00690
IK loss, left hand        0.00771  0.00448
mirror disagreement, mean    32.7      3.8   vendor units of 255
mirror disagreement, worst   166.0     15.0
pinky tip command mean       70.2    204.8
```

Mirror disagreement is the useful invariant: the same gesture made with both
hands must produce the same vendor command, since a G20 slot means the same
thing on each hand. It also removed the thumb asymmetry, so that discrepancy was
length mismatch rather than the differing `thumb_cmc_yaw` axis.

### Abduction polarity

Every vendor abduction slot sets its own finger's lateral angle, and the four
share one positive direction. Commanding all four slots to a single value
therefore swings the whole hand sideways and leaves the finger gaps unchanged,
confirmed on hardware. Spread reaches the hand as fingers holding **opposite**
roll values, which the IK produces on its own.

The URDF agrees: its four `mcp_roll` joints also share the axis `[1,0,0]`, and
sweeping any one across its full 0.34 rad shifts that fingertip by an identical
0.0333 m. One sign per side is therefore the correct structure. Per-finger signs
are actively wrong, because they cancel the opposition between fingers and
collapse a spread gesture into a uniform swing.

Only one fact had to be observed on hardware, because the vendor's radian tables
describe its internal joint convention and say nothing about its relationship to
this URDF:

```text
commanding the left index abduction slot to 255 moves the index toward the THUMB
```

The URDF then fixes both hands. On the left, `+roll` moves a fingertip toward the
thumb side, so 255 must mean `+roll`: not inverted. On the right, `+roll` moves it
toward the little-finger side, so the same anatomical result needs 255 to mean
`-roll`: inverted. Hence `ABDUCTION_INVERTED = {"left": False, "right": True}`,
which comes out exactly opposite to the vendor's own `derict` table for L20. That
table is about the vendor's internal joint sign, not about this URDF, so reading
the vendor code alone can never reveal this.

`test_abduction_polarity_matches_urdf_geometry` re-derives the polarity from the
URDF and forward kinematics, so replacing the assets or hand-editing the table
fails the test rather than silently inverting abduction. `abduction_invert:=true`
flips both sides if a differently wired hand ever needs it, and
`ros2 run linker_hand_bridge slot_probe` re-runs the single observation.

### Remaining known issues

- Abduction still saturates. `mcp_roll` is limited to +/-0.17 rad, narrower than
  an operator's natural finger spread, so ring and pinky reach the outward limit
  after only about 10 mm of operator spread. Neither the length normalization nor
  the sign fix changes this: the range simply is not there, so spread is partly
  clipped rather than reproduced.
- The neutral pose is not centred in the abduction range. With a relaxed hand the
  index slot sits near 38 while ring and pinky sit near 226, so the four fingers
  do not start from a common neutral. Calibrating a per-finger abduction offset
  is unfinished work.
- The vendor's own abduction range is +/-0.26 rad while these URDFs allow only
  +/-0.17, so the mapping currently stretches the URDF range across the full
  vendor range. That maximizes available spread at the cost of a roughly 1.5x
  gain on abduction. Using the vendor range instead would be more faithful and
  less responsive.
- The two URDFs are mirror-consistent in 20 of 21 joints. Only `thumb_cmc_yaw`
  differs, axis `[0,1,0]` with origin rpy `[pi, 0, pi/2]` on the right against
  axis `[-1,0,0]` with no rotation on the left. It no longer produces a visible
  asymmetry, but it means per-joint limits are not guaranteed to carry the same
  anatomical meaning on both sides.
- Retargeting is position-only and has no collision model. It matches landmark
  positions and never checks whether two robot fingers occupy the same space.
- `initial_speed` also raises the force the fingers apply before the firmware
  backs off. Lower it when grasping something fragile.

## Shutdown

Disengage both arms, stop the PICO process, then stop Terminal 2 and Terminal
1. Disable FCI when the workcell is unattended.

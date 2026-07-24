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

## Shutdown

Disengage both arms, stop the PICO process, then stop Terminal 2 and Terminal
1. Disable FCI when the workcell is unattended.

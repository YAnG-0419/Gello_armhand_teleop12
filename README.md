# Franka Upper Body Teleop

Source-neutral teleoperation and data collection for the mounted dual Franka
FR3 workcell. PICO 4 Ultra is the first input implementation. The command
boundary also supports future exoskeleton, GELLO, and replay adapters without
letting a source bypass shared safety checks.

## Architecture

```text
PICO/XR host process -> UDP -> pico_teleop_bridge
                                  |
future source adapters -----------+--> /teleop/arm_commands
                                           |
                                    safety gateway
                                  /         |          \
                         arbitration   validation     dry-run
                                           |
                              /teleop/validated_arm_commands
                                           |
                              /target_robot/joint_commands
                                           |
                                  per-arm splitter
                                           |
                              FR3 impedance controllers
```

Only the safety gateway can publish the hardware command bus. Hardware output
defaults to disabled. It requires fresh measured state, a valid source
session, increasing sequence numbers, an initial target near the measured
pose, FR3 joint limits, and a per-joint slew limit.

## Repository layout

```text
config/                Current workcell configuration
docker/                Minimal ROS Humble/libfranka runtime and Compose graph
ros_ws/src/
  teleop_interfaces    Source-neutral ArmCommand message
  teleop_core          Contract, arbitration, safety gateway, joint splitter
  pico_teleop_bridge   PICO UDP-to-ROS adapter
  teleop_data          Recorder, normalized conversion, replay
  franka_fr3_arm_controllers
teleop_sources/pico/   XR input, Pink IK, MuJoCo model, host tests
scripts/               Build, test, simulation, deployment, data workflows
```

## Initial setup

```bash
cp docker/.env.example docker/.env
# Edit TELEOP_DATA_ROOT if needed.
mkdir -p /home/descfly/franka_teleop_data

./scripts/build.sh
./scripts/setup_pico_env.sh
```

## Simulation and offline tests

```bash
./scripts/run_simulation.sh --headless --mock-xr --duration 2
./scripts/run_simulation.sh --mock-xr
./scripts/start_fake_franka.sh
./scripts/test_offline.sh
./scripts/test_dry_run.sh
```

The MuJoCo model includes the exact dual-arm mount transforms and official
FR3v2 collision meshes, inertias, joint limits, and motor inertias copied
with their license and notice.

`start_fake_franka.sh` separately validates the complete dual `ros2_control`
bring-up, controller loading, realtime scheduling, and CPU affinity using
ROS fake hardware.

## Hardware

Follow [docs/HARDWARE_DEPLOY.md](docs/HARDWARE_DEPLOY.md). The short form is:

```bash
./scripts/start_franka.sh
./scripts/start_pico_dry_run.sh
./scripts/run_pico.sh
```

Do not enable output until the validated topic has been inspected in dry-run.

## Recording

The recorder validates required topics before starting. Camera topics in its
configuration are optional and can be replaced without changing recorder
code.

```bash
./scripts/record.sh
record> start
record> stop
record> save
```

Episodes are stored under `${TELEOP_DATA_ROOT}/episodes/episodeN`.

Convert a raw episode to the versioned, fixed-rate portable schema:

```bash
./scripts/convert.sh \
  /data/episodes/episode0 \
  /data/normalized/episode0.npz
```

The normalized arrays are:

- `timestamp`
- `observation_arm_joint_position` with shape `(frames, 14)`
- `action_arm_joint_position` with shape `(frames, 14)`
- `active_sides` with shape `(frames, 2)`
- `fps`
- `schema_version`

Replay uses the same source arbitration and safety gateway as live input:

```bash
# Stop the live PICO host command stream first.
./scripts/replay.sh /data/normalized/episode0.npz
```

Replay smoothly prepositions from fresh measured state and aborts if robot
state becomes stale. Whether it is dry-run or hardware-enabled is controlled
only by the separately launched safety gateway.

## Adding another input method

See [docs/ADDING_TELEOP_SOURCE.md](docs/ADDING_TELEOP_SOURCE.md). A new
adapter publishes the normalized `ArmCommand` contract. Its device input and
retargeting may differ, but arbitration and hardware safety remain shared.

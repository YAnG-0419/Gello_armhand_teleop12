# Franka Upper Body Teleop

PICO teleoperation, recording, conversion, and replay for the mounted dual
Franka FR3 workcell.

## Structure

```text
PICO -> host IK -> UDP -> ROS adapter -> command gateway -> FR3 controllers
                          ^
                       replay
```

All live and future input methods publish the same `ArmCommand` interface.
Device input and retargeting stay in source-specific packages; robot control
and data tooling do not depend on PICO.

```text
teleop_sources/pico/   PICO input, IK, and MuJoCo simulation
ros_ws/src/            ROS interfaces, control, data, and FR3 controller
config/                Workcell and recording configuration
docker/                ROS Humble runtime
scripts/               Operator commands
```

## Setup and build

```bash
cp docker/.env.example docker/.env
./scripts/build.sh
./scripts/setup_pico_env.sh
```

Run the simulation smoke check:

```bash
./scripts/run_simulation.sh --headless --mock-xr --duration 2
```

For the real workcell, follow [docs/HARDWARE_DEPLOY.md](docs/HARDWARE_DEPLOY.md).

## Configuration

Every setting has one owner; there are no environment or code fallbacks:

- `docker/.env`: data mount, ROS domain, CPU set, and selected workcell file
- `config/current_workcell.yaml`: both real FR3 connections
- `config/pico.yaml`: PICO UDP endpoints, scale, timeout, and rate
- `config/teleop_control.yaml`: command gateway limits and allowed sources
- `ros_ws/src/teleop_data/config/recording.yaml`: recording, conversion, and replay
- `ros_ws/src/franka_fr3_arm_controllers/config/initial_pose.yaml`: captured reset pose

Start with `cp docker/.env.example docker/.env`. All scripts using Docker go
through `scripts/compose.sh`, which reads only that file and rejects missing,
empty, duplicate, unknown, or malformed entries. YAML readers likewise reject
missing and unknown fields.

## Data

The interactive operator provides completion, live status, recording, initial
pose capture, and reset:

```bash
./scripts/operator.sh
teleop> /capture
teleop> /record
teleop> /stop
teleop> /save
teleop> /reset
```

Convert and replay:

```bash
./scripts/convert.sh /data/episodes/episode0 /data/normalized/episode0.npz
./scripts/replay.sh /data/normalized/episode0.npz
```

Stop the live PICO process before replay. Data paths are rooted at
`TELEOP_DATA_ROOT` from `docker/.env`.

## Adding another input device

Add a source adapter that publishes `teleop_interfaces/msg/ArmCommand` on
`/teleop/arm_commands`. Keep device I/O and retargeting inside its own
`teleop_sources/<source>` package. The adapter must not publish the FR3 joint
command topic directly.

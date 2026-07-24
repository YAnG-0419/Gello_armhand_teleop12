# Franka Upper Body Teleop

PICO controller or motion-tracker teleoperation, recording, conversion, and
replay for the mounted dual Franka FR3 workcell.

```text
PICO input -> host IK -> UDP -> ROS adapter -> command gateway -> FR3 controllers
                                ^
                             replay
```

Every input publishes the same `ArmCommand` interface. Device I/O stays in
`teleop_sources/`; robot control and data tooling do not depend on PICO.

## Setup

```bash
cp docker/.env.example docker/.env
./scripts/build.sh
./scripts/setup_pico_env.sh
```

Mock simulation:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/teleop_dual_fr3_mujoco.py \
  --config config/pico.yaml --headless --mock-xr --duration 2
```

For hardware, follow [docs/HARDWARE_DEPLOY.md](docs/HARDWARE_DEPLOY.md).

## Configuration

Each setting has one owner:

- `docker/.env`: data mount, ROS domain, CPU set, selected workcell
- `config/current_workcell.yaml`: real FR3 connections
- `config/pico.yaml`: selected PICO input and all PICO/UDP parameters
- `config/teleop_control.yaml`: command gateway
- `ros_ws/src/teleop_data/config/recording.yaml`: data pipeline
- `ros_ws/src/franka_fr3_arm_controllers/config/initial_pose.yaml`: reset pose

`scripts/compose.sh` validates `docker/.env`, then forwards its arguments
unchanged to `docker compose`. It does not choose services or add ROS options.

## Data CLI

Start the interactive operator with the complete command:

```bash
./scripts/compose.sh run --rm tools ros2 run teleop_data operator \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml \
  --qos /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording_qos.yaml
```

Its commands include `/capture`, `/reset`, `/record`, `/stop`, `/save`,
`/discard`, and `/status`.

Convert and replay:

```bash
./scripts/compose.sh run --rm tools ros2 run teleop_data convert \
  /data/episodes/episode0 /data/normalized/episode0.npz \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml

./scripts/compose.sh run --rm tools ros2 run teleop_data replay \
  /data/normalized/episode0.npz \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml
```

Stop live PICO input before replay.

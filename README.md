# Franka Upper Body Teleop

Monorepo for dual-FR3 teleoperation, recording, conversion, and replay.

```text
PICO input -> retargeting/IK -> UDP -> ROS gateway -> FR3 controllers
                                      ^
                                   replay
```

PICO inputs are selected explicitly on the CLI:

```text
--input controllers
--input motion-trackers
```

The choice is not stored in YAML. Both inputs share the same mapping, IK, UDP,
ROS, and robot-control pipeline.

## Setup

```bash
cp docker/.env.example docker/.env
./scripts/build.sh
./scripts/setup_pico_env.sh
```

The only public scripts are:

- `build.sh`: build the Docker image and ROS workspace
- `setup_pico_env.sh`: create/update the PICO Conda environment

Docker services use ordinary Compose commands from `docker/`; Compose reads
`docker/.env` automatically.

Mock simulation:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/teleop_dual_fr3_mujoco.py \
  --config config/pico.yaml --input mock --headless --duration 2
```

See [docs/HARDWARE_DEPLOY.md](docs/HARDWARE_DEPLOY.md) for real PICO and FR3
commands.

## Configuration ownership

- `docker/.env`: host paths, ROS domain, CPU allocation, workcell selection
- `config/current_workcell.yaml`: FR3 addresses and namespaces
- `config/pico.yaml`: controller and motion-tracker settings, mapping, IK, and UDP
- `config/teleop_control.yaml`: ROS command gateway
- `ros_ws/src/teleop_data/config/recording.yaml`: data pipeline
- `ros_ws/src/franka_fr3_arm_controllers/config/initial_pose.yaml`: reset pose

Missing or unknown YAML fields are errors.

# Franka Upper Body Teleop

Monorepo for dual-FR3 teleoperation, recording, conversion, and replay.

```text
arm pose source -> mapping/IK -> UDP -> ROS gateway -> FR3 controllers
operator engage --------^
operator engage -> hand worker -> retargeting -> hand bridge
```

Arm inputs are selected explicitly on the CLI:

```text
--arm-source controllers
--arm-source motion-trackers
--arm-source vive-trackers --vive-config config/vive.yaml
```

The choice is not stored in YAML. VIVE Trackers are the operational default; PICO remains optional. All arm inputs share the same mapping, IK, UDP, ROS, and robot-control pipeline.

## Input boundaries

`DualFr3HardwareTeleop` receives an arm pose source, operator state, and optional hand controller; it does not construct or import a device adapter. A new arm device such as VIVE implements the small `ArmPoseSource` protocol and is assembled by an entrypoint.

PICO adapters share one explicitly owned `PicoSession`. Hand retargeting runs in a local worker, so hand SDK reads and solves cannot delay the 100 Hz arm loop. Arm and hand command paths are independent; the only intentional runtime coupling is the shared per-side engagement and the policy that an arm-input safety fault disengages its hand.

## Setup

```bash
cp docker/.env.example docker/.env
./scripts/build.sh
./scripts/setup_pico_env.sh
```

The only public scripts are:

- `build.sh`: build the Docker image and ROS workspace
- `setup_pico_env.sh`: create/update the host teleoperation Conda environment
- `start_orbbec_viewer.sh`: open the compatible SDK v2 Viewer when ROS is stopped
- `run_teleop.sh`: start the default VIVE-arm/MANUS-hand host operator
- `run_pico_teleop.sh`: start the optional PICO-arm/MANUS-hand host operator
- `export_lerobot.sh`: export complete arm, hand, and RGB-D bags to LeRobot

Docker services use ordinary Compose commands from `docker/`; Compose reads `docker/.env` automatically.

Mock simulation:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/teleop_dual_fr3_mujoco.py \
  --config config/pico.yaml --arm-source mock --headless --duration 2
```

See [docs/HARDWARE_DEPLOY.md](docs/HARDWARE_DEPLOY.md) for hardware operation, recording, export, and replay. VIVE setup and its read-only connectivity tool are documented in [teleop_sources/vive/README.md](teleop_sources/vive/README.md). See [docs/HANDOVER.md](docs/HANDOVER.md) for current state and next work. MANUS implementation notes are in [teleop_sources/manus/README.md](teleop_sources/manus/README.md).

## Configuration ownership

- `docker/.env`: host paths, ROS domain, CPU allocation, workcell selection
- `config/current_workcell.yaml`: FR3 addresses and namespaces
- `config/pico.yaml`: controller and motion-tracker settings, mapping, IK, and UDP
- `config/teleop_control.yaml`: ROS command gateway
- `ros_ws/src/teleop_data/config/recording.yaml`: data pipeline and per-side recorded hand models
- `ros_ws/src/franka_fr3_arm_controllers/config/initial_pose.yaml`: reset pose

Missing or unknown YAML fields are errors.

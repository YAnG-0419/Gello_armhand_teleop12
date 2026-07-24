# Dual-FR3 deployment

Current workcell:

- left FR3: `172.16.0.3`
- right FR3: `172.16.0.2`
- host: `enp6s0`, `172.16.0.6/24`
- ROS domain: `0`

Keep the emergency stop reachable.

## Setup

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
cp docker/.env.example docker/.env
./scripts/build.sh
./scripts/setup_pico_env.sh
```

Review every value in `docker/.env` and `config/pico.yaml`. Before a session,
activate both arms and FCI in Franka Desk, then verify:

```bash
ping -c 2 172.16.0.3
ping -c 2 172.16.0.2
```

## Choose PICO input

Set one selector in `config/pico.yaml`:

```yaml
input:
  type: controllers       # or: motion_trackers
```

Controller mode uses each grip as an independent hold-to-run clutch. After
startup or stale data, release the grip once before reacquiring that arm.

Motion-tracker mode uses keyboard activation. First identify the serials:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/list_motion_trackers.py \
  --config config/pico.yaml
```

Move one tracker at a time and fill
`input.motion_trackers.serials.{left,right}`. The mounting transforms initially
use identity; calibrate them later if rotation creates unwanted translation.

Keyboard controls are:

- `Space`: toggle both arms
- `L` / `R`: toggle one arm
- `X`: disable both arms
- `Q`: disable both arms and exit

Both modes can be checked in MuJoCo before starting the robots:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/teleop_dual_fr3_mujoco.py \
  --config config/pico.yaml
```

Verify that PICO forward/right/up maps to robot `+X/-Y/+Z`.

## Run

Start XRoboToolkit PC Service and the selected PICO stream.

Terminal 1 — FR3 controllers:

```bash
./scripts/compose.sh up franka-control
```

Terminal 2 — enabled output:

```bash
./scripts/compose.sh up teleop-control pico-bridge
```

For dry-run output, use:

```bash
./scripts/compose.sh up teleop-control-dry-run pico-bridge
```

Terminal 3 — selected PICO input:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml
```

On the first robot run, acquire one arm at a time and test a small translation
before rotation or bimanual motion.

## Operator and data

```bash
./scripts/compose.sh run --rm tools ros2 run teleop_data operator \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml \
  --qos /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording_qos.yaml
```

Before `/reset`, release both controller grips or press `X` in tracker mode.
Stop the PICO Python process before replay.

## Shutdown

Disengage both arms, stop Terminal 3, Terminal 2, then Terminal 1. Disable FCI
when the workcell is unattended.

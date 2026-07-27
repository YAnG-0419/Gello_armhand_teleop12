# Dual-FR3 full-pipeline runbook

Commands for FR3 arms, LinkerHand G20 hands, PICO/MANUS teleoperation,
Orbbec RGB-D, recording, export, and replay.

## Current MANUS right hand + right arm

The operator is one process. PICO supplies the right-arm motion tracker and
MANUS supplies the right-hand skeleton. The same `R`, `Space`, and `X`
activation state gates both command streams; do not run
`teleop_full_thumb.py` separately.

### Terminal 1 — ROS services

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up hand-control franka-control teleop-control pico-bridge
```

### Terminal 2 — unified operator

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers \
  --hand-source right-only-manus
```

Only the right tracker is required; the left LinkerHand holds its default
pose. The terminal prints per-side tracker and hand-send status once per
second. `H` still homes both arms.

## Full teleop and recording

### Terminal 1 — ROS services

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up
```

### Terminal 2 — PICO operator

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers --hand-source pico
```

`--arm-source motion-trackers` is the recommended choice: measured quiet EE tremor is
~5x lower than with `--arm-source hand-roots` (3.4 vs 16.8 mrad on the 2026-07-26
vs 2026-07-27 sessions), because the tracker has no optical-skeleton wrist
noise. Keep both trackers in the headset's view; occlusion freezes them and
disengages that arm. Use `hand-roots` only when the task forces the trackers
out of view.

For a synchronized arm-jitter and hand-retargeting diagnostic trial, add both
logs:

```bash
RUN_DIR=/home/descfly/franka_teleop_data/diagnostics/$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"

conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers --hand-source pico \
  --debug-log "$RUN_DIR/ee_jitter_with_hands.jsonl" \
  --hand-debug-log "$RUN_DIR/hand_fidelity.jsonl"
```

First repeat the same static-hand arm trial without `--hand-source` (and without
`--hand-debug-log`) to isolate arm behavior. Store it as
`"$RUN_DIR/ee_jitter_no_hands.jsonl"` so all recordings from one comparison
remain together. Analyze the recordings offline in the same terminal:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/analyze_follow_log.py \
  "$RUN_DIR/ee_jitter_with_hands.jsonl"

conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/analyze_hand_retarget_log.py \
  "$RUN_DIR/hand_fidelity.jsonl"
```

Do not use `/tmp` for hardware evidence intended for later comparison. The
logger truncates an existing filename, so create a new timestamped `RUN_DIR`
for every diagnostic session.

Controls:

- `Space`: toggle both sides
- `L` / `R`: toggle one side
- `X`: disengage both sides
- `O`: open disengaged hands
- `H`: disengage, open hands, and reset arms
- `Q`: disengage and exit

### Terminal 3 — episode recorder

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools ros2 run teleop_data operator \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml \
  --qos /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording_qos.yaml
```

```text
/status
/record
/stop
/save
```

Start recording when `/status` reports:

```text
arms=2/2 | record_topics=12/12 | reset=ready
```

Each episode contains:

- left/right FR3 state and action;
- left/right LinkerHand state and action;
- RGB, 16-bit depth, and both camera-info topics.

Episodes are saved under:

```text
/home/descfly/franka_teleop_data/episodes/episodeN
```

## Replay and export

### Camera replay

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools ros2 run teleop_data replay_camera \
  /data/episodes/episode0 \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml
```

The replay topics use the `/replay/camera/` prefix. Use `--speed 0.5` or
`--speed 2.0` to change playback speed.

### LeRobot export

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
./scripts/export_lerobot.sh \
  /data/episodes/episode0 /data/episodes/episode1 \
  --output /data/lerobot/my_dataset \
  --task "describe the demonstrated task" \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml \
  --fps 10
```

State and action are each 54-dimensional: 14 Franka joints followed by the
left and right 20-slot G20 values.

### Robot replay

This command moves both arms and both hands:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools ros2 run teleop_data replay \
  /data/lerobot/my_dataset \
  --episode-index 0 \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml
```

Disengage PICO, verify the preposition path, and keep the emergency stop
reachable. Arm commands pass through the safety gateway.

## Other commands

### Reset arms

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools \
  ros2 service call /reset_to_initial_pose std_srvs/srv/Trigger '{}'
```

The reset is joint-space interpolation, not collision planning.

### Motion trackers with PICO optical hands

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers --hand-source pico
```

List tracker serial numbers:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/inspect_motion_trackers.py
```

### Controllers

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source controllers
```

### Hands only

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up hand-control
```

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_hands.py
```

Do not run `teleop_hands.py` and `teleop_dual_fr3.py` together.

### Orbbec only

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up -d orbbec
docker compose logs -f orbbec
docker compose run --rm tools ros2 topic list | grep '^/camera/'
```

Expected image topics:

```text
/camera/color/image_raw
/camera/depth/image_raw
```

Do not run OrbbecViewer while the ROS camera service owns the camera. See
[ORBBEC_CAMERA.md](ORBBEC_CAMERA.md) for viewer and network recovery.

## Hand diagnostics

### Thumb configurations

View `open`, `tip-only`, and `base-and-tip` without moving hardware:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/inspect_thumb_configuration.py
```

Send one preset through the hand bridge:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/inspect_thumb_configuration.py \
  --side right --preset tip-only --send-hardware
```

Use `--side left` for the left hand or `--preset base-and-tip` for combined
thumb-base and thumb-tip flexion. Stop PICO teleop before using this command.
It does not publish FR3 commands.

### PICO skeleton

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/inspect_hand_tracking.py \
  --duration 30 --log /tmp/pico_thumb_check.jsonl
```

Close `RobotLinuxDemo` and PICO teleop before running the inspector.

## Hardware reference

```text
left FR3    172.16.0.3
right FR3   172.16.0.2
host        enp6s0: 172.16.0.6/24, 192.168.1.53/24
Orbbec      192.168.1.10:8090

can0  0x28  left   LHT20-010-502-L-B-1-D
can1  0x27  right  LHT20-010-556-R-B-1-D
```

Bring up CAN:

```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
```

Do not reconfigure `enp6s0` during an active FCI session.

## Verification

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  pytest -q teleop_sources/pico/tests

cd ros_ws/src/linker_hand_bridge
PYTHONPATH=. python3 -m pytest -q test/test_core.py
```

Shutdown order: disengage PICO, stop the PICO process, stop Compose, then
disable FCI.

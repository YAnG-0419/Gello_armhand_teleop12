# Dual-FR3 full-pipeline runbook

Commands for FR3 arms, mixed LinkerHand G20/O30i hands, PICO/MANUS teleoperation,
Orbbec RGB-D, recording, export, and replay.

## Current MANUS right hand + right arm

The operator is one process. PICO supplies the right-arm motion tracker and
MANUS supplies the right-hand skeleton. The same `R`, `Space`, and `X`
activation state gates both command streams; do not run
`teleop_full_thumb.py` separately.

The right-hand-only MANUS/O30i path has been physically validated. The
integrated arm plus mixed left-G20/right-O30i configuration is the next
real-world test and is not yet marked validated.

### Terminal 1 — ROS services

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
# hand-control now defaults to the physical pair: left G20 + right O30i.
docker compose up franka-control teleop-control pico-bridge hand-control

# Dry-run model and packet validation only:
cd /home/descfly/hsc/franka_upper_body_teleop
ros2 launch linker_hand_bridge hands.launch.py \
  sides:=both left_model:=g20 right_model:=o30i enabled:=false
```

The connected `a8fa:8598` CANFD Analyser uses the packaged `libcanbus`
transport and does not create a `can1` interface. Run the O30i driver through
the privileged Compose `hand-control` service so it can access the USB device.
The alternative transparent SocketCAN adapter can still select
`o30_transport:=socketcan`.
The read-only launch publishes uncalibrated vendor ticks on
`/linker_hand_o30i/raw_state`; it deliberately does not publish those values
as canonical radians.

The current physical tests use the vendor's normalized full-range mapping:
each URDF joint lower limit maps to tick 0 and its upper limit maps to tick
255. The wrapper supplies this mapping and acknowledges it explicitly.
Optional measured endpoint vectors follow the canonical O30i URDF order:
`thumb_cmc_roll, thumb_cmc_yaw, thumb_mcp, thumb_ip`, then roll, pitch, PIP,
and DIP for index, middle, ring, and pinky. Use them only after a systematic
per-device calibration:

```bash
export O30_TICKS_AT_LOWER='20-comma-separated-measured-ticks'
export O30_TICKS_AT_UPPER='20-comma-separated-measured-ticks'
teleop_sources/manus/scripts/run_o30_robot.sh
```

The O30i node verifies the device model and right-hand identity before it can
enable motors. It also requires fresh, in-calibration position feedback before
the first command. Command gaps are normal in teleoperation - a disengaged
side simply stops streaming and the hand holds position. Feedback loss or a
failed command disables all 20 joints until fresh feedback and a new command
arrive; only a rejected disable is terminal and requires a node restart.

### Right-hand-only MANUS/O30i test

Validate MANUS and retargeting without hardware output first:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools ros2 launch linker_hand_bridge \
  hand_bridge.launch.py sides:=right right_model:=o30i enabled:=false

# In a second terminal:
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/manus/scripts/teleop_o30i.py
```

The hand-only operator starts disengaged. `Space` or `R` enables right-hand
following, `X` stops sending, `O` requests an open pose while disengaged, and
`Q` exits. For a real test, replace the dry-run bridge with the calibrated
`hands.launch.py sides:=right right_model:=o30i enabled:=true` invocation
through the privileged `hand-control` service.

For the real two-terminal workflow, use the wrapper scripts. The robot wrapper
uses the vendor's normalized full-range mapping (URDF lower limit = tick 0,
URDF upper limit = tick 255) unless per-device endpoint vectors are supplied:

```bash
# Terminal 1: robot-side O30i driver and safety bridge
cd /home/descfly/hsc/franka_upper_body_teleop
teleop_sources/manus/scripts/run_o30_robot.sh

# Terminal 2: MANUS source, initially disengaged
cd /home/descfly/hsc/franka_upper_body_teleop
teleop_sources/manus/scripts/run_o30_manus.sh
```

The robot wrapper defaults to the O30i profile's `12.0 rad/s` slew ceiling.
The MANUS wrapper defaults to output EMA `alpha=0.85`; set
`O30_FILTER_ALPHA=1.0` to disable that EMA for latency comparison.
Per-device endpoint calibration can still override the normalized mapping:

```bash
export O30_TICKS_AT_LOWER='20-comma-separated-ticks'
export O30_TICKS_AT_UPPER='20-comma-separated-ticks'
```

### Terminal 2 — unified operator

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers \
  --hand-source right-only-manus
```

Tracker presence is per-side at runtime: startup needs at least one tracker,
and each arm engages only while its own tracker is live — a missing tracker's
side refuses to engage with the reason, and its dropout while disengaged does
not disturb the other arm. The left G20 holds its default pose because there
is no left glove yet. `H` still homes both arms. This software path exists,
but its complete mixed-hardware real-world validation remains pending.

The operator terminal is a TUI by default: a status header updated once per
second, an operator pane showing only the feedback for keys you pressed, and
a process pane that captures everything the SDKs and libraries print —
including native C-level output. `--ui plain` restores ordinary line output;
use it when redirecting the terminal to a file.

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

With the mixed G20/O30i configuration, state and action are each
54-dimensional: 14 Franka joints, the left 20-slot G20 vector, and the right
20-joint O30i URDF-radian vector. The converter derives hand widths, names,
models, and limits independently per side from `recording.yaml`.
Replay preposition speed is also configured per side because the G20 driver
state uses ticks while the O30i driver state uses URDF radians.

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

PYTHONPATH=ros_ws/src/linker_hand_bridge:ros_ws/src/linker_hand_ros2_sdk \
  python3 -m pytest -q \
  ros_ws/src/linker_hand_bridge/test/test_core.py \
  ros_ws/src/linker_hand_bridge/test/test_o30i_profile_limits.py \
  ros_ws/src/linker_hand_ros2_sdk/test/test_o30i_contract.py \
  ros_ws/src/linker_hand_ros2_sdk/test/test_o30i_contract_limits.py \
  ros_ws/src/linker_hand_ros2_sdk/test/test_o30i_transport.py
```

Shutdown order: disengage PICO, stop the PICO process, stop Compose, then
disable FCI.

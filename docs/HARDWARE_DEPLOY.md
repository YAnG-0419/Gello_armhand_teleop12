# Dual-FR3 full-pipeline runbook

Commands for FR3 arms, mixed LinkerHand G20/O30i hands, PICO/MANUS teleoperation,
Orbbec RGB-D, recording, export, and replay.

## MANUS hands + arms

The operator is one process. PICO supplies the arm motion trackers and
MANUS supplies both hand skeletons (`--hand-source manus`, the
`run_teleop.sh` default; `--hand-sides right` degrades to the old
right-only behavior). One activation state per side gates that side's arm
and hand together, driven from the operator GUI; do not run
`teleop_full_thumb.py` separately.

Each glove only delivers frames once its calibration file exists in
`teleop_sources/manus/config` (`Calibration_left.mcal` /
`Calibration_right.mcal`) - the bridge silently drops uncalibrated gloves,
and only the right file exists as of 2026-07-29. Check what is connected
and streaming (owns the MANUS client; stop teleop first):

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/manus/scripts/inspect_manus_gloves.py
```

The right-hand-only MANUS/O30i path has been physically validated. The
bimanual MANUS configuration (left G20 through the same fixed-opposition
L20 profile as the validated PICO left hand) is implemented but not yet
hardware-validated.

### Terminal 1 — ROS services

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
# hand-control now defaults to the physical pair: left G20 + right O30i.
docker compose up franka-control teleop-control pico-bridge hand-control
```

Hand hardware output is always on (the dry-run `enabled` flag was removed
2026-07-29); `/linker_hand_bridge/{side}/mapped_command` mirrors every
command for inspection.

Bringup applies the raised collision thresholds automatically (a one-shot
node in `robot_control.launch.py`; nothing else sets them). Verify both
`accepted the collision thresholds` lines in the franka-control log on every
bringup - a rejection or timeout is loud but does not stop the launch. The
safety gateway also gates commands on measured external joint torques; see
[CONTACT_IK_VALIDATION.md](CONTACT_IK_VALIDATION.md) for calibration.

The connected `a8fa:8598` CANFD Analyser uses the packaged `libcanbus`
transport and does not create a `can1` interface. Run the O30i driver through
the privileged Compose `hand-control` service so it can access the USB device.
The alternative transparent SocketCAN adapter can still select
`o30_transport:=socketcan`. The O30i node also publishes uncalibrated
vendor ticks on `/linker_hand_o30i/raw_state`; it deliberately does not
publish those values as canonical radians.

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

### MANUS hands-only test (no arms)

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up hand-control

# In a second terminal - pick --sides left, right, or both:
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/manus/scripts/teleop_manus_hands.py --sides left
```

The hand-only operator starts disengaged. `L`/`R` toggles one side and
`Space` toggles the selected sides together, `X` stops sending, `O`
requests an open pose while disengaged, and `Q` exits. Do not run it
beside `run_teleop.sh` - one MANUS client at a time.

For the two-terminal workflow, use the wrapper scripts. The robot wrapper
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
scripts/run_teleop.sh
```

The wrapper creates a fresh RUN_DIR, prints it, and records both debug logs.
Prefer it over pasting the multi-line command: a clipboard missing its final
newline once left the command pending at the prompt, and the next paste
glued onto its log argument, burying the recording.

Tracker presence is per-side at runtime and never blocks the session: with
zero trackers the operator still starts, Open hands and Home work against
the live robot, and each arm becomes engageable the moment its own tracker
appears. A missing side refuses to engage with the reason, and
its dropout while disengaged does not disturb the other arm. The left G20
holds its default pose because there is no left glove yet.

The operator process is headless: one unified backend serving a JSON-TCP
control port (default `127.0.0.1:5590`), with the PySide6 GUI as its
frontend (installed in the conda base env; re-install with
`conda activate base && pip install -i
https://pypi.tuna.tsinghua.edu.cn/simple -e teleop_sources/gui`). Run
`teleop-operator-gui` next to the operator: it reconnects automatically
and shows the once-per-second status line plus the operator feedback log;
the terminal keeps plain process output. Engage starts a side following
your motion; disengage stops following and the arm holds (not a power
cut - the physical e-stop stays the top-level stop).

## Full teleop and recording

### Terminal 1 — ROS services

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up
```

### Terminal 2 — PICO operator

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
scripts/run_teleop.sh --hand-source pico
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

Controls (operator GUI buttons; any home first disengages the session and
the untargeted arm holds in place):

- Engage left / right: toggle one side's arm-and-hand following
- DISENGAGE ALL: stop both sides immediately
- Open hands: open the disengaged hands
- Home left / right / both: open that side's hand and reset that arm
- Ctrl-C in the operator terminal: disengage and exit

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

Per-side variants move only that arm (`/reset_to_initial_pose/left`,
`/reset_to_initial_pose/right`); `/capture_initial_pose` saves the current
measured pose as the new home. The reset is joint-space interpolation, not
collision planning.

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

Assign trackers to sides without reading any labels - the script prompts
you to move only the left hand, then only the right, and works out which
serial is which; `--write` stores the result in `config/pico.yaml`
(comments preserved, file re-validated). Stop teleop first; this owns the
single PICO SDK client:

```bash
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/calibrate_tracker_sides.py --write
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

# MANUS hand input

MANUS supplies calibrated hand skeletons to the unified operator. Each dynamic side is retargeted to its configured robot model and sent as named joint radians to `linker_hand_bridge`; arm and hand activation are coupled per side.

## Requirements

- Only one MANUS CoreSDK client may run at a time.
- Each glove needs `teleop_sources/manus/config/Calibration_left.mcal` or `Calibration_right.mcal`.
- The standard models are the physically calibrated left G20 pose-anchor profile and the right O30i solver.

Build the native bridge:

```bash
teleop_sources/manus/scripts/build.sh
```

Inspect connected gloves with teleop stopped:

```bash
conda run --no-capture-output -n franka-teleop-pico python teleop_sources/manus/scripts/inspect_manus_gloves.py
```

## Standard operator

Use the repository wrapper:

```bash
scripts/run_teleop.sh
```

It starts the unified PICO motion-tracker and bimanual MANUS backend. Use the PySide6 operator GUI to engage or disengage each side, home arms, and open hands as independent per-side actions.

## Hands-only

Start the hand drivers:

```bash
cd docker
docker compose up hand-control
```

Then start MANUS:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output -n franka-teleop-pico python teleop_sources/manus/scripts/teleop_manus_hands.py --sides both
```

Controls are `L`/`R`, `Space`, `X`, `O`, and `Q`.

The left thumb uses continuous interpolation between a stable ordinary-pose
anchor and a physically tuned thumb-index pinch anchor. Curl endpoints and the
four finger endpoints come from the labelled MANUS recording. Do not replace
this policy with unconstrained per-frame CMC IK: the L20 URDF predicts a 35 mm
gap at the physical G20 contact pose and previously produced basin flips.

## O30i behavior

The right O30i driver maps URDF lower and upper limits to normalized ticks 0 and 255 unless `O30_TICKS_AT_LOWER` and `O30_TICKS_AT_UPPER` provide measured endpoints. It verifies model and handedness, requires fresh calibrated feedback before commanding, holds across normal command gaps, and disables on feedback loss or command failure.

## Accuracy recording

The endpoint protocol tests four explicit expectations: a fully open human hand
maps to an open robot, four fully curled fingers map to their closed range,
thumb-index pinch reaches the configured contact pose, and a fully curled thumb
reaches its closed range. For the left G20, physical contact—not L20 URDF FK—is
the pinch ground truth. Stop the normal operator first, then record without
hardware:

```bash
conda run --no-capture-output -n franka-teleop-pico \
  python teleop_sources/manus/scripts/record_manus_accuracy.py \
  --output /home/descfly/franka_teleop_data/manus_accuracy/run1.jsonl \
  --sides both

conda run --no-capture-output -n franka-teleop-pico \
  python teleop_sources/manus/scripts/analyze_manus_accuracy.py \
  /home/descfly/franka_teleop_data/manus_accuracy/run1.jsonl

# Evaluate new solver code against the exact same recorded landmarks:
conda run --no-capture-output -n franka-teleop-pico \
  python teleop_sources/manus/scripts/analyze_manus_accuracy.py \
  --replay-current /home/descfly/franka_teleop_data/manus_accuracy/run1.jsonl
```

The recorder sends only to a private local UDP sink. Its v2 rows include the
complete 25-keypoint MANUS frame and orientations, source sequence/timestamp,
canonical landmarks, raw and filtered named radians, robot targets/FK,
calibration hashes, solver configuration, and the would-be UDP identity.

For an end-to-end hardware trial, use the hands-only operator with
`--debug-log FILE` and simultaneously record these ROS topics:

```text
/linker_hand_bridge/left/mapped_command
/linker_hand_bridge/right/mapped_command
/cb_left_hand_control_cmd
/cb_right_hand_control_cmd
/cb_left_hand_state
/cb_right_hand_state
```

The normal teleop-data recorder already captures the four `cb_*` command/state
topics. Bridge command headers carry `stream_id:sequence`, matching the host
JSONL transport identity, so solver output, slew-limited command, and feedback
can be aligned later.

## Retargeting policy

Do not add hidden operator-specific gains. Derive calibration from repeatable,
labelled multi-pose recordings. Analyze recordings offline before changing
objectives or limits.

The skeleton pipeline above is the single supported MANUS path.

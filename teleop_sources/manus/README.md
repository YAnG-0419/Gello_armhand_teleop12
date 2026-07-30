# MANUS hand input

MANUS supplies calibrated hand skeletons to the unified operator. Each dynamic side is retargeted to its configured robot model and sent as named joint radians to `linker_hand_bridge`; arm and hand activation are coupled per side.

## Requirements

- Only one MANUS CoreSDK client may run at a time.
- Each glove needs `teleop_sources/manus/config/Calibration_left.mcal` or `Calibration_right.mcal`.
- The standard models are left G20 through the L20 profile and right O30i.
- The standard left G20 path uses fixed thumb opposition. The full left-thumb solver is experimental and opt-in.

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

Controls are `L`/`R`, `Space`, `X`, `O`, and `Q`. Add `--left-thumb full` only to evaluate the experimental full-thumb solver.

## O30i behavior

The right O30i driver maps URDF lower and upper limits to normalized ticks 0 and 255 unless `O30_TICKS_AT_LOWER` and `O30_TICKS_AT_UPPER` provide measured endpoints. It verifies model and handedness, requires fresh calibrated feedback before commanding, holds across normal command gaps, and disables on feedback loss or command failure.

Use `teleop_sources/manus/scripts/run_o30_robot.sh` and `run_o30_manus.sh` only for the standalone two-terminal diagnostic workflow; do not run them beside the unified operator.

## Retargeting policy

Do not add hidden operator-specific gains. Derive calibration from repeatable multi-pose recordings. `hand_fidelity.jsonl` contains replayable landmarks and solver diagnostics; analyze it offline before changing objectives or limits.

The old standalone C++ ergonomics-angle adapter remains only for legacy dual-G20 diagnostics and does not support O30i.

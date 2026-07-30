# Repository handover

Current state as of 2026-07-30. Use [HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md) for operation and [ORBBEC_CAMERA.md](ORBBEC_CAMERA.md) for camera recovery; older investigations belong in git history.

## Workcell

```text
left FR3    172.16.0.3          right FR3  172.16.0.2
host        enp6s0: 172.16.0.6/24, 192.168.1.53/24
Orbbec      192.168.1.10:8090
left hand   G20  can0 0x28      right hand O30i libcanbus USB a8fa:8598
PICO tracker ids: config/pico.yaml
```

## Current status

- Standard startup is `docker compose up franka-control teleop-control pico-bridge hand-control`, then `scripts/run_teleop.sh`, then `python teleop_sources/gui/operator_gui.py`.
- Arms are settled; contact torque gating and collision thresholds are hardware-validated. Do not retune without reading the relevant git history.
- The false-stale PICO regression is resolved and hardware-verified. Motion is parsed first, published atomically, and considered fresh only when the local callback sequence advances; no native parsing exception may cross the vendor callback boundary.
- Do not restore the former multi-getter consistency loop or cached-snapshot engagement grace. An invalid atomic snapshot disengages immediately.
- Bimanual MANUS works. The right O30i uses full-thumb retargeting; the left G20 remains fixed-opposition in the standard operator path.
- The experimental left full-thumb solver is available only through `teleop_manus_hands.py --left-thumb full`; validate it before changing the default.

## Invariants

- Never run two PICO clients or two MANUS clients simultaneously.
- GUI loss, tracker loss while engaged, stale robot state, gateway rejection, or an invalid tracker snapshot disengages affected control.
- `seq` is local receipt of a parsed Motion object; `ts` is only a vendor payload timestamp. Rising `callback_errors` means SDK fields or frames were rejected. `n=0` does not prove the physical trackers are inactive.
- Position and rotation liveness are checked independently. Do not weaken freshness, jump, speed, torque, slew, or acquisition checks to hide a fault.
- Only the safety gateway publishes the FR3 command bus. The host operator stays ROS-free.
- Home and replay move hardware. Preserve logs before shutting down after any reflex or unexplained fault.

## Next work

- Validate the opt-in left full-thumb solver on recorded pinch, wrap, and lateral grasps, then on hardware.
- Improve MANUS/O30i fidelity only from recordings that include landmarks, solved radians, commands, and feedback.

## Verification

PICO: `conda run -n franka-teleop-pico pytest -q teleop_sources/pico/tests`.

Gateway: `PYTHONPATH=ros_ws/src/teleop_core python3 -m pytest -q ros_ws/src/teleop_core/test/`.

Data lives under `/home/descfly/franka_teleop_data/`; `hand_fidelity*.jsonl` is replayable retargeting input.

# Repository handover

Current state as of 2026-07-30. Use [HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md) for operation and [ORBBEC_CAMERA.md](ORBBEC_CAMERA.md) for camera recovery; older investigations belong in git history.

## Workcell

```text
left FR3    172.16.0.3          right FR3  172.16.0.2
host        enp6s0: 172.16.0.6/24, 192.168.1.53/24
Orbbec      192.168.1.10:8090
left hand   G20  can0 0x28      right hand O30i libcanbus USB a8fa:8598
VIVE hand tracker ids: config/vive.yaml    PICO fallback ids: config/pico.yaml
```

## Current status

- Standard startup is `docker compose up franka-control teleop-control vive-bridge hand-control`, then `scripts/run_teleop.sh`, then `python teleop_sources/gui/operator_gui.py`. PICO is the optional fallback through `pico-bridge` and `scripts/run_pico_teleop.sh`; never run both bridges.
- Arms are settled; contact torque gating and collision thresholds are hardware-validated. Do not retune without reading the relevant git history.
- The false-stale PICO regression is resolved and hardware-verified. Motion is parsed first, published atomically, and considered fresh only when the local callback sequence advances; no native parsing exception may cross the vendor callback boundary.
- Do not restore the former multi-getter consistency loop or cached-snapshot engagement grace. An invalid atomic snapshot disengages immediately.
- Bimanual MANUS works. The left G20 uses full thumb retargeting against the vendor L20 URDF. CMC yaw/roll/pitch and coupled MCP/IP flex are jointly optimized using the O30i-style position, segment-direction, and activated excess-distance terms, with an 18 mm recorded contact deadzone and 10x distance weighting. There is no thumb-index pose anchor. Warm start, activation release, a 0.35 rad/tick thumb trust region, output EMA, and joint limits remain active. `left_o30i_style_pinch.jsonl` and the 2026-07-31 physical check confirmed accurate pinch and acceptable thumb rotation. The right O30i retains calibrated open/curl endpoints, contact-biased index pinch, and a smoothly activated cooperative thumb-middle pinch anchor.
- The left model is `assets/linkerhand_l20/left/linkerhand_l20_left.urdf`, copied from `linker-bot/linkerhand-urdf` commit `735145e8843f44d85c1464725e6971ba97a6258e`. The prior claim that the G20 geometry differed from this L20 model was an inference from model-vs-contact observations, not a verified vendor fact; the operator states G20 should use this L20 URDF.
- Arm sources, operator state, and hands are injected into the hardware coordinator. PICO SDK ownership is explicit and hand retargeting runs outside the arm loop; add future arm adapters without importing them into the coordinator.

## Invariants

- Never run two PICO clients or two MANUS clients simultaneously.
- GUI loss, tracker loss while engaged, stale robot state, gateway rejection, or an invalid tracker snapshot disengages affected control.
- `seq` is local receipt of a parsed Motion object; `ts` is only a vendor payload timestamp. Rising `callback_errors` means SDK fields or frames were rejected. `n=0` does not prove the physical trackers are inactive.
- Position and rotation liveness are checked independently. Do not weaken freshness, jump, speed, torque, slew, or acquisition checks to hide a fault.
- Only the safety gateway publishes the FR3 command bus. The host operator stays ROS-free.
- Home and replay move hardware. Preserve logs before shutting down after any reflex or unexplained fault.

## Next work

- Physically validate the new index-middle contact anchors on each hand before marking them final.
- Improve hand fidelity only from recordings that include landmarks, solved radians, commands, and feedback; do not add posture-specific thumb anchors when the continuous optimizer can represent the motion.

## Verification

PICO: `conda run -n franka-teleop-pico pytest -q teleop_sources/pico/tests`.

Gateway: `PYTHONPATH=ros_ws/src/teleop_core python3 -m pytest -q ros_ws/src/teleop_core/test/`.

Data lives under `/home/descfly/franka_teleop_data/`; `hand_fidelity*.jsonl` is replayable retargeting input.

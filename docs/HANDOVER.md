# Repository handover

State as of 2026-07-29. Runbook: [HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md);
camera: [ORBBEC_CAMERA.md](ORBBEC_CAMERA.md). History lives in the git log.

## Setup

```text
left FR3    172.16.0.3          right FR3  172.16.0.2
host        enp6s0: 172.16.0.6/24, 192.168.1.53/24
Orbbec      192.168.1.10:8090
left hand   G20  can0 0x28      right hand O30i libcanbus USB a8fa:8598
PICO trackers: left PC2310MLL5060501G, right PC2310MLL5290914G
```

Bringup: `docker compose up franka-control teleop-control pico-bridge
hand-control` (hand-control = left G20 + right O30i, enabled). Operator:
`scripts/run_teleop.sh` - fresh RUN_DIR plus both debug logs; extra flags
pass through and later flags win (`--hand-source pico`, `--ui plain`).

Status:

- Arms are settled (impedance gains, FOH interpolation, 200 Hz broadcasters,
  tracker input; quiet EE tremor 3.4 mrad / 0.83 mm). Do not retune without
  reading the git history; hand-roots input is the occlusion fallback.
- The mixed stack ran on hardware 2026-07-29 (teleop + 3.4 min recording;
  engage/disengage cycles validated the recoverable O30i watchdog). Pending:
  per-side tracker fault drills, feel-check of the 2026-07-29 retargeting
  change (segment-direction + pinch terms; offline: MCP deficit -> ~0 deg,
  pinch at touch 17 -> 5.7 mm), a left-G20 grasp in mixed mode.
- Hand path: MANUS -> canonical landmarks -> O30i solver (2.3 ms mean) ->
  UDP :5570 -> bridge (250 ms watchdog, per-model slew) -> drivers. PICO
  optical hands remain the G20-only path. Bimanual MANUS not implemented.

## Research agenda

### 1. Contact-rich tolerance: why does table contact red-light the arm?

Audited offline 2026-07-29; evidence and the operator plan:
[CONTACT_IK_VALIDATION.md](CONTACT_IK_VALIDATION.md). The collision-behavior
script never ran (nothing invokes it) and would have sent zero acceleration
thresholds; fixed, response-checked, now a manual bringup step. Structural
cause: the gateway slew walks the command into an obstacle while the stiff
impedance turns deviation into torque - a per-joint deviation cap is
prepared in the gateway (`max_command_deviation`, default off); trials pend.

### 2. IK transparency: unreachable pose, or IK failure?

Instrumented 2026-07-29: `ik.py` classifies every step (ok / joint-limit /
speed-clamp / workspace), the STATE line shows each side's worst cause once
per second, and `follow-debug.v3` logs it per tick. Replaying all 14
sessions: past deficits were mostly j7 at its limit and workspace edges.
Pending: an operator staged-reach check (stretch, j7 stop, fast sweep).

### 3. Collision awareness (low priority)

The bimanual IK knows nothing about self- or table collision; only the
Franka reflex intervenes. Candidate: pink collision barriers, validated in
the mujoco harness before hardware.

### 4. Hands (parked)

- MANUS->O30i precision: record all four layers (landmarks, radians,
  commands, feedback ticks) over a repeatable pose set before more tuning.
- G20: no G20 URDF exists (models are L20); fixed-opposition thumb is the
  answer - full-thumb retargeting is research, not a bugfix.
- Force: touch sensors and motor current are unread; a force-gated press is
  implementable against the existing bridge.

## Invariants and traps

- One SDK client owns PICO input; never run RobotLinuxDemo beside it.
- `detected=[]` with the SDK connected = the headset stopped sending (seen
  after 2-day service uptime). Diagnose: `ss -tnp | grep 60061`, then
  `inspect_motion_trackers.py` alone, then headset re-enter / service restart.
- `isActive` + array-change liveness stay mandatory (SDK serves cached poses).
- Trackers are per-side, never blocking: zero trackers still starts (O/H
  work); absent sides cannot engage; loss while engaged disengages all.
- O30i driver: command gaps hold position; feedback loss disables
  recoverably; only a rejected disable is terminal.
- The host process stays ROS-free (`env_guard.py`); `config/pico.yaml` is
  validated by both the host parser and the pico bridge launch file.
- `H` and robot replay move hardware; keep PICO disengaged.
- After any reflex, save `docker compose logs franka-control` BEFORE
  `down`; treat the first engagement after a restart as suspect
  (unresolved 2026-07-26 violent right-arm reflex, logs lost).

## Tests

Pico suite (110): `pytest -q teleop_sources/pico/tests` in the conda env.
ROS-side (59): see HARDWARE_DEPLOY verification. teleop_data (20) runs in
the tools container. Python changes need only a service restart.

## Data

Under `/home/descfly/franka_teleop_data/`: `hand_coexistence.jsonl` (only
tracker-vs-skeleton dataset); `diagnostics/20260726_*` (tracker sessions,
incl. the 8.7 min screwdriver run); `diagnostics/20260729_143116` (first
mixed session; drove the retargeting change). `hand_fidelity*.jsonl` files
are exact retarget inputs, replayable offline.

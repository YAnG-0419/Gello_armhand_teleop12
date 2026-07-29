# Repository handover

State as of 2026-07-29 evening. Runbook: [HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md);
camera: ORBBEC_CAMERA.md. History lives in the git log.

## Setup

```text
left FR3    172.16.0.3          right FR3  172.16.0.2
host        enp6s0: 172.16.0.6/24, 192.168.1.53/24
Orbbec      192.168.1.10:8090
left hand   G20  can0 0x28      right hand O30i libcanbus USB a8fa:8598
PICO tracker ids: config/pico.yaml; re-assign via calibrate_tracker_sides.py
```

Bringup: `docker compose up franka-control teleop-control pico-bridge
hand-control`. Operator: `scripts/run_teleop.sh` (headless backend) +
`python teleop_sources/gui/operator_gui.py` (PySide6, TCP :5590, base env).
All engage/home/open actions are GUI buttons; GUI loss disengages all.

Status:

- Arms are settled (quiet EE tremor 3.4 mrad / 0.83 mm); do not retune
  without reading the git history. Contact is RESOLVED (auto thresholds +
  torque gating, [CONTACT_IK_VALIDATION.md](CONTACT_IK_VALIDATION.md)); IK
  failures are classified live and per tick (follow-debug.v4). Initial pose
  recaptured 2026-07-29; per-side home services exist. Gateway verdicts
  stream back (protocol v2): a rejected engage shows its reason in the GUI.
- Bimanual MANUS ran on hardware 2026-07-29 evening; both gloves stream
  (probe: inspect_manus_gloves.py), engage/disengage cycles work after the
  re-engage-deadlock fix. Pending: per-side tracker fault drills, feel-check
  of the O30i thumb change, left-G20 grasp quality (agenda 1).
- Hand path: MANUS -> canonical landmarks -> per-side solver (right O30i
  full-thumb; left G20 = L20 profile, FIXED thumb opposition) -> UDP :5570
  -> bridge (250 ms watchdog, per-model slew) -> drivers.

## Research agenda

### 1. G20 thumb: replace the fixed opposition with real retargeting

The G20 thumb's CMC yaw/roll are frozen at THUMB_OPPOSITION_YAW_ROLL
(hand_retarget.py ~L66-86; right operator-tuned 2026-07-26, left an
UNTUNED copy - the mirrored URDF may want a different roll); only the curl
is live (bend 0.25-1.30 rad -> full pitch+flex). Known costs: 16 mm
contact gap mimicking the human root (2026-07-26 recordings); one fixed
orientation cannot serve pinch, wrap, and lateral grasps. Solve it
systematically, not by re-tuning constants:

- Prior art: the O30i solver's segment-direction + pinch terms took thumb
  MCP deficit to ~0 deg and pinch-at-touch from 17 to 5.7 mm offline
  (manus_teleop/o30i_retarget.py). The same method - explicit objectives
  replayed against recorded landmarks - should drive the G20/L20 thumb.
- Define grasp metrics first (thumb-tip vs finger-tip distances,
  opposition plane angle); evaluate on replayed `hand_fidelity*.jsonl` and
  a NEW recorded pose set (pinch / power wrap / lateral), then feel.
- Constraint: no G20 URDF exists; models are L20. Decide explicitly
  whether the L20 thumb model is the limit or the mapping is.
- Tools: analyze_hand_retarget_log.py, inspect_thumb_configuration.py,
  tune_thumb_opposition.py, diagnose_o30i_retarget.py (metric-loop model).

### 2. Collision awareness (low priority)

The bimanual IK knows nothing about self- or table collision; only the
Franka reflex intervenes. Candidate: pink collision barriers, validated in
the mujoco harness before hardware.

### 3. Hands, parked

- MANUS->O30i precision: record all four layers (landmarks, radians,
  commands, feedback ticks) over a repeatable pose set before more tuning.
- Force: touch sensors and motor current are unread; a force-gated press is
  implementable against the existing bridge.

## Invariants and traps

- One SDK client owns PICO input, one owns MANUS; never run RobotLinuxDemo
  or a second operator/hands-only script beside a live session.
- `detected=[]` = the headset stopped sending; probe with
  inspect_motion_trackers.py alone. isActive + array-change liveness stay
  mandatory. Trackers are per-side; loss while engaged disengages all.
- O30i driver: command gaps hold; feedback loss disables recoverably; only a
  rejected disable is terminal. O30i silent on CANFD: probe_o30i_identity.py.
- The host stays ROS-free (env_guard.py); pico.yaml is validated by host
  parser and bridge launch; .msg changes need a colcon rebuild.
- Home buttons and robot replay move hardware. After any reflex or bug,
  save `docker compose logs franka-control teleop-control` BEFORE `down`
  (two incidents lost logs); first engagement after a restart is suspect.

## Tests

Pico (124): `pytest -q teleop_sources/pico/tests` in franka-teleop-pico.
Gateway/protocol (10): `PYTHONPATH=ros_ws/src/teleop_core python3 -m pytest
-q ros_ws/src/teleop_core/test/`. Container suites: HARDWARE_DEPLOY.

## Data

Under `/home/descfly/franka_teleop_data/`: `hand_coexistence.jsonl`;
`diagnostics/20260726_*` (tracker sessions, thumb-gap evidence);
`diagnostics/20260729_*` (mixed sessions). `hand_fidelity*.jsonl` are exact
retarget inputs, replayable offline - the substrate for agenda 1.

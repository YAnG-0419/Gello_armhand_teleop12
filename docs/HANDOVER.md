# Repository handover

State as of 2026-07-30. Runbook: [HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md);
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
  failures are classified live and per tick (follow-debug.v6, with per-tick
  SDK feed forensics: frame_ts/ts/seq/age/callback_errors/ok/n). Initial pose
  recaptured 2026-07-29; per-side home services exist. Gateway verdicts
  stream back (protocol v2): a rejected engage shows its reason in the GUI.
- The false-stale PICO regression is RESOLVED and hardware-verified
  2026-07-30. A malformed decimal in any SDK JSON section could throw
  `std::invalid_argument` through the vendor C callback, silently stopping
  its receive loop while Python and gRPC still looked alive. Motion is now
  parsed first and atomically published; no exception crosses the callback
  boundary. Python reads one locked motion snapshot and uses its local
  callback sequence for feed liveness. Do not restore the former multi-getter
  consistency loop or cached-snapshot engagement grace.
- Bimanual MANUS ran on hardware 2026-07-29 evening; both gloves stream
  (probe: inspect_manus_gloves.py), engage/disengage cycles work after the
  re-engage-deadlock fix. Pending: per-side tracker fault drills, feel-check
  of the O30i thumb change, and left-G20 full-thumb validation (agenda 1).
- Hand path: MANUS -> canonical landmarks -> per-side solver (right O30i
  full-thumb; left G20 = L20 profile, FIXED thumb opposition) -> UDP :5570
  -> bridge (250 ms watchdog, per-model slew) -> drivers.

## Research agenda

### 1. G20 full-thumb validation

The full left-G20 thumb solver now exists with activation release and a
0.35-rad-per-tick trust region. It is intentionally opt-in only in the
hands-only diagnostic (`--left-thumb full`); the operator pipeline remains
fixed-opposition by default and is unchanged unless the mode is explicitly
selected. Validate the new mode systematically before changing that default:

- The implementation uses explicit segment-direction, tip, and pinch
  objectives replayed against recorded landmarks. Confirm that activation
  release prevents trust-region wedging across changing grasps.
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
- `seq` is local receipt of a successfully parsed Motion object; it, not the
  vendor payload timestamp, is feed liveness. `callback_errors` increasing
  identifies rejected SDK fields/frames. `n=0` means the latest parsed Motion
  object contained no usable trackers; it does not by itself prove the PICO
  hardware is inactive. Position and rotation liveness checks stay mandatory.
  Trackers are per-side; loss while engaged disengages all.
- O30i driver: command gaps hold; feedback loss disables recoverably; only a
  rejected disable is terminal. O30i silent on CANFD: probe_o30i_identity.py.
- The host stays ROS-free (env_guard.py); pico.yaml is validated by host
  parser and bridge launch; .msg changes need a colcon rebuild.
- Home buttons and robot replay move hardware. After any reflex or bug,
  save `docker compose logs franka-control teleop-control` BEFORE `down`
  (two incidents lost logs); first engagement after a restart is suspect.

## Tests

Pico (132): `pytest -q teleop_sources/pico/tests` in franka-teleop-pico.
Gateway/protocol (10): `PYTHONPATH=ros_ws/src/teleop_core python3 -m pytest
-q ros_ws/src/teleop_core/test/`. Container suites: HARDWARE_DEPLOY.

## Data

Under `/home/descfly/franka_teleop_data/`: `hand_coexistence.jsonl`;
`diagnostics/20260726_*` (tracker sessions, thumb-gap evidence);
`diagnostics/20260729_*` (mixed sessions). `hand_fidelity*.jsonl` are exact
retarget inputs, replayable offline - the substrate for agenda 1.

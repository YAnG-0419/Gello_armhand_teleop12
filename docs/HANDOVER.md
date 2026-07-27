# Repository handover

State as of the night of 2026-07-26. Operator commands are in
[HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md); camera recovery is in
[ORBBEC_CAMERA.md](ORBBEC_CAMERA.md).

The operator is satisfied with FR3 arm control; it is settled and should not
be re-tuned casually. The active research front is the hands. This document
keeps the operational essentials plus what bears on the hand research; the
full history of the 2026-07-26 arm-control and jitter work lives in the git
log (commits `c545a9b`..`7960098`, each self-explanatory).

## Working setup

- Current experiment: right arm from its PICO motion tracker and right
  LinkerHand from MANUS, owned by one operator process
  (`--hand-source right-only-manus`); the left LinkerHand holds its default
  pose. The earlier PICO optical skeleton path (`--hand-source pico`) remains
  hardware-validated end to end on both G20s, including pressing an electric
  screwdriver button (with a workaround, see the force section).
- Tracker vs hand-root tradeoff, measured: the tracker's position is
  sub-millimetre with optical fix and carries no in-band wander, so the arms
  are smooth; but its position comes from the headset cameras seeing the
  tracker, and a wrist occlusion or leaving the view freezes it (the
  freeze/jump guards then disengage that arm). Current practice: the
  operator keeps both trackers in view at all times. Hand-root input
  (`--arm-source hand-roots`) survives occlusion and side-grasps but carries
  7-34 mrad of session-dependent optical wrist noise in the 0.5-3 Hz band
  that reaches the end effector as visible tremor; it is the fallback, not
  the default. Direct A/B on the same control stack (2026-07-26 tracker
  session vs 2026-07-27 hand-root session): quiet EE tremor 3.4 mrad /
  0.83 mm vs 16.8 mrad / 2.57 mm - the residual arm jitter is entirely an
  input-side property.
- Arm-side control state (do not change without reading the git history):
  franka_ros2 example impedance gains, first-order-hold command
  interpolation in the 1 kHz controller, 200 Hz state broadcasters, host
  position EMA 0.20 s / rotation 0.10 s fixed. Dead ends already measured:
  stronger fixed smoothing, error-adaptive rotation EMA with thresholds
  below the noise floor, skeleton landmark averaging, re-softened gains.

Hardware:

```text
left FR3    172.16.0.3
right FR3   172.16.0.2
host        enp6s0: 172.16.0.6/24, 192.168.1.53/24
Orbbec      192.168.1.10:8090
can0        left G20,  0x28
can1        right G20, 0x27
PICO trackers: left PC2310MLL5060501G, right PC2310MLL5290914G
```

Teleop command (create a FRESH RUN_DIR every session - the logger truncates
existing files and a reused shell variable has already destroyed two
recordings):

```bash
RUN_DIR=/home/descfly/franka_teleop_data/diagnostics/$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers --hand-source pico \
  --debug-log "$RUN_DIR/ee_jitter.jsonl" \
  --hand-debug-log "$RUN_DIR/hand_fidelity.jsonl"
```

`docker compose up -d` starts everything; `docker compose up -d hand-control`
brings up only the hand stack, output enabled. Its operational defaults live
in `docker/compose.yaml`; launching `hands.launch.py` directly keeps output
disabled.

## Hand system: current implementation

- Skeletons are converted to 21 canonical hand-frame landmarks
  (`hand_landmarks.py`); this is the seam a different hand-pose source would
  plug into. The four ordinary fingers go through the per-finger optimizer
  in `hand_retarget.py`.
- The thumb runs in fixed-opposition mode: (cmc yaw, roll) locked per side
  (`THUMB_OPPOSITION_YAW_ROLL`, right hardware-tuned to (1.10, 0.52), left
  still the copied values), and the operator's thumb bend, normalized over
  `THUMB_CURL_BEND_RANGE` (0.25-1.30 rad), drives cmc pitch + the coupled
  MCP/IP flex across their full ranges. Kinematic basis: at roll 0 the G20
  thumb's yaw, pitch, MCP, and IP axes are parallel. Retune live with
  `scripts/hardware/tune_thumb_opposition.py --side left|right` (adjusts
  yaw/roll/curl through the running bridge, prints the constant to paste
  back). Omitting `thumb_opposition_fixed` restores the full thumb solver.
- The bridge requests speed 255 and per-finger max torque (thumb 250,
  fingers 200) at startup; the vendor driver never initializes G20 speed or
  torque on its own. Lower for fragile objects. Bridge keeps its 250 ms
  watchdog, 30 Hz cap, 1500 unit/s slew; gesture EMA alpha 0.7.
- Software latency skeleton-to-emitted-qpos is ~0 ms at 30 Hz solves;
  perceived finger slowness is downstream (motors, slew ~85 ms per
  half-swing, 30 Hz send cap).

### MANUS right-hand MVP

`teleop_sources/manus` is a standalone C++ MANUS CoreSDK adapter: it reads
the glove's calibrated ergonomics angles and emits the existing 21-name L20
UDP contract, dynamic right hand plus all-zero left default. It is print-only
without `--send` and stops output when MANUS data is stale; build and dry-run
commands are in its README. Use the bundled Metaglove Pro calibration; this
glove family rejects the sibling reference's non-Pro file. Hardware-validated
2026-07-27: `MetagloveProHaptics` at ~93.5 Hz with no discarded frames, and a
10 s right-G20 run at 30 Hz with clean packets and CAN.

`teleop_sources/manus/scripts/teleop_full_thumb.py` is an experimental
raw-skeleton path through the canonical 21-landmark solver
(`thumb_opposition_fixed=None`); its `--send` mode is a hand-only diagnostic.
A cautious hardware test drove every thumb coordinate dynamically with clean
packets/CAN, but deep-opposition tip error reached ~33 mm; keep it
experimental until operator feel is compared directly.

Abduction polarity: the bridge's derived per-side baseline is correct for the
unified MANUS mode and `abduction_invert` stays false. Offline comparison of
the 2026-07-27 session logs (PICO 12:27 vs MANUS 13:55) shows identical
skeleton chirality, so both sources drive the same retargeter convention. An
earlier `abduction_invert:=true` in Compose was a wrong-layer workaround
(operator-confirmed inverted finger gaps on 2026-07-27 evening); if a future
ergonomics-based sender (e.g. the C++ adapter's `Spread()`) shows closed gaps
on spread, fix that sender's sign, not the bridge.

The hardware entrypoint is `teleop_dual_fr3.py --hand-source
right-only-manus`: one process owns both SDK clients and ticks MANUS from the
arm loop, so `R`, `Space`, and `X` gate the right arm and hand together while
the left hand holds its default pose. Only the right tracker is required;
per-side status prints once per second. `H` stays a global workcell HOME and
resets both arms regardless of source mode.

## Research agenda

### 1. Hand pose source (PICO skeleton quality is the suspected limit)

Operator intends to explore multi-camera vision-based hand tracking (no
occlusion) or Manus gloves. Measured facts to reuse: PICO skeleton noise is
a whole-pose coherent wander (7-31 mrad quiet RMS at the wrist, varying by
session with hand position in the headset view; per-landmark jitter ~3 mm on
every finger equally, so landmark averaging does not help); tracking quality
is the input floor for any retargeting improvement.

Integration: there are two seams, pick per source. The lowest one is the
hand bridge's UDP qpos packet (`hand_stream.py`, port 5570): anything that
produces the 21-name URDF joint vector drives the hands directly, bypassing
our retargeting entirely - the natural path for joint-space sources like
gloves. The higher seam is the 21 canonical landmarks
(`hand_landmarks.py`): a vision source that plugs in there reuses the
existing retargeting unchanged. Either way a liveness signal is required
(the watchdog stops the hand, nothing more), wrist pose for the arms can
stay on the motion trackers, and multiple hand-teleop methods can coexist
as alternative senders to the same bridge.

### 2. Retargeting fidelity

Not yet studied carefully for the four fingers; the thumb was studied and
its solver history should not be repeated: mimicking the human thumb root on
this heterogeneous mechanism is a conflicted objective (a reachability
oracle touched thumb-index tips exactly while the converged solver left a
16 mm pinch gap; freeing the coupled flex, 20x iterations, and fading the
direction terms all failed - the last made it worse). The fixed-opposition
mode is the working answer for the thumb; treat any return to full-thumb
retargeting as research, not a bugfix. Two known confounds to resolve first:

- Model identity: the physical hands are G20 (`G20(工业版)` in the vendor
  SDK, dedicated CAN class) but every kinematic model here and in sibling
  repos is an L20 URDF; no G20 URDF exists on this machine. Offline
  model-based pose choices repeatedly disagreed with hardware feel. Get a
  G20 URDF from the vendor, or verify axis-by-axis with the tuner against
  the L20 model in `inspect_thumb_configuration.py`.
- Hand representation: the somehand reference (read-only sibling) uses the
  same constraint weights but a whole-hand solve over relative directions
  and hand-scaled distances; our per-finger staged solve with absolute
  canonical positions is the main structural difference. Any representation
  change should be validated offline first - the replay harness exists
  (`hand_fidelity.jsonl` logs are exact retarget inputs;
  `analyze_hand_retarget_log.py` summarizes; see git history for the replay
  methodology).

### 3. Hand force control

G20 exposes no position-loop gains - the full CAN register map offers only
position, speed, per-finger torque cap, faults, temperature thresholds, and
sensor queries. In a stalled contact the press force equals the torque cap
(measured: during button presses the curl command saturates, so the cap is
the whole story). Unexplored and promising: the hand has per-finger normal
force (0x90), tangential force, matrix touch sensors, and motor current
readback, none of which our stack reads today. A force-controlled press
(ramp curl until measured normal force reaches a target) is implementable
against the existing bridge without new hardware. Mechanical fact: press
force scales as torque/lever-arm - contact near the thumb root is worth
2-3x over the fingertip.

### 4. Electric screwdriver primitives

Current manual technique: fixed-opposition grasp, thumb-pad press, plus a
workaround - the four fingers counter-press from the other side because the
thumb alone is marginal even at torque 250. Primitive ideas worth
prototyping: a keyboard-triggered "trigger pulse" (scripted curl press and
release), grasp presets per tool, and a force-gated press built on the
touch sensors from item 3. The keyboard request pattern to copy is the
existing `O`/`H` flow (`take_requests` in the input classes, serviced in
`hardware.py`); the hand pipeline's `request_open` is the template for a
scripted hand action.

## Safety and process invariants

- Do not run `RobotLinuxDemo` beside a Python XRoboToolkit client; one SDK
  client owns both arm and hand input.
- `isActive` and array-change detection stay mandatory: the SDK serves
  plausible cached skeletons after tracking loss.
- Do not run `teleop_hands.py` and `teleop_dual_fr3.py` together; the thumb
  tuner refuses to start beside either.
- Do not run OrbbecViewer while the ROS Orbbec service owns the camera; do
  not reconfigure `enp6s0` during an active FCI session.
- The host process must remain ROS-free (`env_guard.py`); `config/pico.yaml`
  is validated by both the host parser and the pico bridge launch file.
- Robot replay and `H` reset cause physical motion; keep PICO disengaged.
- Unresolved incident: one violent right-arm motion + reflex stop on the
  first run after a docker restart (2026-07-26 evening); logs were lost to
  `docker compose down`. Save `docker compose logs franka-control` before
  taking the stack down after any reflex, and treat the first engagement
  after a restart as suspect.
- Another automation agent (a Cursor sandbox) has been seen inspecting this
  machine; unexplained service state changes may be someone else operating.

## Tests and builds

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  pytest -q teleop_sources/pico/tests            # 90
cd ros_ws/src/linker_hand_bridge
PYTHONPATH=. python3 -m pytest -q test/test_core.py   # 30
```

teleop_data tests (15) run in the tools container; see HARDWARE_DEPLOY.md.
The workspace is volume-mounted with symlink-install: config and Python
changes need only a service restart; only the C++ controller needs
`docker/build_workspace.sh --packages-select franka_fr3_arm_controllers`.

## Still unvalidated (outside the research agenda)

- Recording a complete FR3/G20/RGB-D episode, LeRobot export, and
  four-device replay - untouched by the 2026-07-26 work and the largest
  remaining block before data collection.
- Left-hand thumb opposition values; the startup torque request's actual
  effect on press force (motor current is readable live via the
  `electric_current` setting command while pressing).
- Per-side arm-gates-hand behavior and the `O`/`H` keyboard flows.

## Local data

The closed jitter investigation's recordings were deleted 2026-07-26 night.
What remains, all from the final configuration (trackers + fixed-opposition
thumb, evening of 2026-07-26):

```text
/home/descfly/franka_teleop_data/hand_coexistence.jsonl        PICO skeletons + tracker poses recorded
                                                               simultaneously, 10 Hz - the only
                                                               tracker-vs-skeleton dataset
/home/descfly/franka_teleop_data/diagnostics/20260726_201920/  tracker sessions of increasing length;
/home/descfly/franka_teleop_data/diagnostics/20260726_204055/  204055 is the 8.7 min screwdriver
/home/descfly/franka_teleop_data/diagnostics/20260726_210453/  button-press session
/home/descfly/franka_teleop_data/diagnostics/20260726_212245/
/home/descfly/franka_teleop_data/diagnostics/20260726_213109/  hand_fidelity_rescued.jsonl was recovered
                                                               from a mangled RUN_DIR paste
```

Diagnostic recordings, not calibration ground truth. `hand_fidelity*.jsonl`
files contain exact retarget inputs and are replayable offline.

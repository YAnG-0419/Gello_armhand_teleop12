# Repository handover

State as of 2026-07-29. Operator commands are in
[HARDWARE_DEPLOY.md](HARDWARE_DEPLOY.md); camera recovery is in
[ORBBEC_CAMERA.md](ORBBEC_CAMERA.md).

The operator is satisfied with FR3 arm control; it is settled and should not
be re-tuned casually. The active research front is the hands. This document
keeps the operational essentials plus what bears on the hand research; the
full history of the 2026-07-26 arm-control and jitter work lives in the git
log (commits `c545a9b`..`7960098`, each self-explanatory).

## Working setup

- Current hand experiment: right MANUS glove to right O30i. The hand-only
  path is physically validated for connectivity, all-finger correspondence,
  and responsive motion. The integrated operator path
  (`--arm-source motion-trackers --hand-source right-only-manus`) exists but
  has not yet been physically validated with the mixed left-G20/right-O30i
  stack. In that mode the left G20 holds its default pose; it is not driven
  by an absent left glove.
- The earlier PICO optical skeleton path (`--hand-source pico`) remains
  hardware-validated end to end on the former dual-G20 setup, including
  pressing an electric screwdriver button. It intentionally does not target
  O30i. PICO motion trackers are arm inputs and are independent of the hand
  pose source.
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
USB         right O30i, libcanbus device a8fa:8598, request ID 0x01
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

The operator terminal runs a three-pane TUI by default (status, operator
feedback, captured process output); `--ui plain` restores line output.

`docker compose up -d hand-control` now brings up the physical pair: left G20
on can0 plus right O30i through libcanbus USB, with the validated 0/255 tick
mapping acknowledged. `run_o30_robot.sh` remains the right-O30i-only wrapper
for hand experiments. Launching `hands.launch.py` directly keeps output
disabled unless `enabled:=true` is supplied.

## Hand system: current implementation

The hand path is model-profiled independently per side. The target
configuration is `left_model:=g20`, `right_model:=o30i`. A device profile
supplies command names and width, bounds, fixed slots, mapping, home pose,
maximum publish rate, feedback validation, and startup settings. Host packets
carry a model tag, and a mismatch with the configured side is rejected. Legacy
untagged MANUS packets remain accepted only for the configured G20 profile.

The MANUS source registers a right O30i retargeter; it still reads only the
right glove. Bimanual MANUS is future work. PICO optical hand tracking remains
G20-only. The bridge and offline data profiles register O30i using canonical
URDF radians. An unknown model fails at startup rather than falling through to
G20. Dataset conversion supports unequal left/right vector widths and writes
normalized v3 model/joint metadata, while replay still reads earlier dual-G20
v2 files.

The supplied O30i description
(`/home/descfly/Downloads/O30i_urdf_0706`) is enough to define the kinematic
side of that profile: it contains separate left/right URDFs with 20 independent
revolute joints, no mimic joints, radians, and per-joint limits. Its canonical
order is four thumb joints followed by four joints for each of index, middle,
ring, and pinky. It must not be passed through `L20Retargeter`, which assumes a
21-joint L20 description, coupled distal joints, and a different thumb chain.

The updated `/home/descfly/Downloads/litchi_hardware-main` includes the real
O30i HOP CAN-FD driver. The required controller source was copied into this
repository with provenance. Its physical protocol uses 20 uint8 ticks in a
type-grouped order; the project-owned driver is the sole radians/tick boundary
and publishes canonical URDF names and radians on ROS. PICO-to-O30 remains
intentionally out of scope.

The physical right hand reports identity
`LHO30i-01.1-012-R-Z-3-A`, firmware `0.0.3`. It is connected through the
vendor `libcanbus` USB transport, not SocketCAN. The O30i node does not enable
motors on connection. Before the first command it requires the reported model
and right-hand identity, mapping acknowledgement, and fresh in-range position
feedback. Command gaps are normal (a disengaged side stops streaming; the
hand holds). Feedback loss or a failed command disables all joints until
fresh feedback and a new command arrive; only a rejected disable is terminal.

For recording, "layout" means the model identifier plus the ordered joint
names, units, bounds, and vector width used for each state/action. The O30i URDF
settles this layout. The currently validated runtime mapping linearly maps each
joint's URDF lower/upper limit to vendor ticks 0/255. This is a general,
model-level mapping, not an operator-specific compensation. Measured
per-device endpoint vectors can override it later if accuracy data shows that
the full vendor range is not the physical joint range.

Vendor setting traffic is now isolated per side. Each driver remaps its global
`/cb_hand_setting_cmd` subscription to `/cb_{side}_hand_setting_cmd`, preventing
one model from consuming the other model's speed or torque request.

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

### MANUS right hand and O30i

The primary O30i path reads the calibrated 25-keypoint right MANUS skeleton,
converts it to canonical landmarks, solves the independent 20-joint O30i URDF,
and sends named radians to the model-aware bridge. The older standalone C++
ergonomics-angle adapter remains a G20-only diagnostic.

The first O30i ordinary-finger implementation used the wrong link
correspondence and also mutated a shared landmark array while normalizing
finger lengths. That made fingertips curl at rest. Both structural bugs are
fixed: proximal/middle/distal/tip landmarks now target the corresponding O30i
chain and each finger is normalized independently. A recorded real-glove
replay reduced IK loss by about 84%; solve time was 1.56 ms average and
2.32 ms p95.

Physical validation on 2026-07-28/29 confirmed correct finger correspondence.
The robot-side slew ceiling is now 12 rad/s and the MANUS output EMA defaults
to `alpha=0.85`; the operator reports the response is good. A temporary pinky
PIP/DIP gain was evaluated and then removed because it encoded one observed
operator/pose mismatch rather than a justified model-level transform. There
is currently no per-finger gain compensation in the O30i retargeter. Slight
O30i motion jitter has been observed but not yet localized to MANUS input,
retargeting, command transport, or hardware feedback.

Abduction polarity: the bridge's derived per-side baseline is correct for the
unified MANUS mode and `abduction_invert` stays false. Offline comparison of
the 2026-07-27 session logs (PICO 12:27 vs MANUS 13:55) shows identical
skeleton chirality, so both sources drive the same retargeter convention. An
earlier `abduction_invert:=true` in Compose was a wrong-layer workaround
(operator-confirmed inverted finger gaps on 2026-07-27 evening); if a future
ergonomics-based sender (e.g. the C++ adapter's `Spread()`) shows closed gaps
on spread, fix that sender's sign, not the bridge.

The validated hand-only entrypoints are
`teleop_sources/manus/scripts/run_o30_robot.sh` and
`teleop_sources/manus/scripts/run_o30_manus.sh`. The integrated entrypoint is
`teleop_dual_fr3.py --arm-source motion-trackers --hand-source
right-only-manus`: one process owns both SDK clients and ticks MANUS from the
arm loop, so `R`, `Space`, and `X` gate the right arm and hand together while
the left hand holds its default pose. Tracker presence is per-side at
runtime: startup needs at least one tracker, a side whose tracker is absent
simply cannot engage (the keyboard refuses with the reason), a disengaged
side's tracker dropout never disturbs the other arm, and losing a tracker
WHILE engaged disengages everything, exactly like a freeze. `H` remains a
global workcell HOME and resets both arms.

## Research agenda

### 1. Mixed-hand real-world validation

The next integration task is a physical run with left G20 and right O30i:

- PICO motion trackers drive the arms.
- A right MANUS glove drives right O30i.
- With only the right glove, left G20 remains at its default pose.
- Bimanual MANUS, where a left glove drives left G20 and the right glove drives
  right O30i, is a later implementation and validation task. It is not
  implemented as of this handover.

The robot-side bringup for the mixed pair now exists: the Compose
`hand-control` service owns left G20 and right O30i together, and the host
drives whichever arms have live trackers (per-side at runtime, no flag).
What remains is the physical run itself: test activation, stale-input
behavior, and independent per-side status on both hands. Do not describe this
as validated until it has run on both physical hands.

### 2. Systematic MANUS-to-O30i precision

Do not add per-finger gains from a single pose observation. First record
synchronized data at four layers: raw MANUS landmarks, retargeted radians,
bridge-emitted commands, and O30i feedback ticks/radians. Use a repeatable pose
set including straight hand, isolated MCP flexion, hook flexion, relaxed fist,
full fist, thumb opposition, and finger spread. This separates:

- glove calibration and operator anatomy,
- human-to-robot kinematic retargeting,
- EMA/slew/transport lag,
- O30i endpoint mapping, backlash, and hardware control.

The general solution should improve the model/objective or mapping using
multi-pose, multi-operator evidence. If one teleoperator still needs a better
fit, add an explicit named calibration profile derived from their recorded
poses, rather than embedding anonymous constants in the retargeter.

### 3. Hand pose source

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

### 4. G20 retargeting fidelity

Not yet studied carefully for the four fingers; the thumb was studied and
its solver history should not be repeated: mimicking the human thumb root on
this heterogeneous mechanism is a conflicted objective (a reachability
oracle touched thumb-index tips exactly while the converged solver left a
16 mm pinch gap; freeing the coupled flex, 20x iterations, and fading the
direction terms all failed - the last made it worse). The fixed-opposition
mode is the working answer for the thumb; treat any return to full-thumb
retargeting as research, not a bugfix. Two known confounds to resolve first:

- Model identity: the physical hands are G20 (industrial model in the vendor
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

### 5. Hand force control

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

### 6. Electric screwdriver primitives

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
  pytest -q teleop_sources/pico/tests            # 96
PYTHONPATH=ros_ws/src/linker_hand_bridge:ros_ws/src/linker_hand_ros2_sdk \
  python3 -m pytest -q \
  ros_ws/src/linker_hand_bridge/test/test_core.py \
  ros_ws/src/linker_hand_bridge/test/test_o30i_profile_limits.py \
  ros_ws/src/linker_hand_ros2_sdk/test/test_o30i_contract.py \
  ros_ws/src/linker_hand_ros2_sdk/test/test_o30i_contract_limits.py \
  ros_ws/src/linker_hand_ros2_sdk/test/test_o30i_transport.py  # 59
```

teleop_data tests (20) run in the tools container; see HARDWARE_DEPLOY.md.
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

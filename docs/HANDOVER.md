# Repository handover

State as of 2026-07-25, end of the PICO hand-integration session. The previous
handover described Linker Hand integration as future work; it is now done and
verified on hardware. `HARDWARE_DEPLOY.md` remains the canonical operator
runbook and carries most per-component facts; this document carries the system
state, the one open investigation, and what the next agent should do first.

## What works, verified on hardware

- Dual-FR3 arm teleoperation from PICO wrist motion trackers, clutch-relative,
  through the container stack. Unchanged in architecture from the previous
  handover, with the fixes listed below.
- PICO optical 26-joint hand tracking driving two LinkerHand G20 hands
  (left `LHT20-010-502-L-B-1-D` on can0 at 0x28, right `LHT20-010-556-R-B-1-D`
  on can1 at 0x27, different firmware 1.0.9/1.0.10). Both real hands followed
  recorded and live gestures; the operator confirmed retargeting quality is
  good for ordinary poses.
- Arms and hands together from one operator process, one SDK client, hands
  retargeted inline at most one side per 100 Hz tick (measured p99 impact
  +2.5 ms, zero missed deadlines).
- Deployment is two terminals: `docker compose up` (franka-control,
  teleop-control, pico-bridge, hand-control; fake-franka-control is behind a
  compose profile) and the operator process
  (`teleop_dual_fr3.py --config config/pico.yaml --input motion-trackers
  [--hands] [--debug-log PATH]`). Hands-only: `docker compose up hand-control`
  plus `teleop_hands.py`.
- 68 host tests (`conda run -n franka-teleop-pico pytest -q
  teleop_sources/pico/tests`) and 30 bridge tests
  (`cd ros_ws/src/linker_hand_bridge && PYTHONPATH=. python3 -m pytest -q
  test/test_core.py`) pass. Container builds via `./scripts/build.sh`.

Everything is on `main`, pushed to
`git@github.com:hesic73/franka_upper_body_teleop.git` (through commit
`e9b903d`). The repository is fully self-contained: the vendor Linker SDK is
vendored at `ros_ws/src/linker_hand_ros2_sdk` (upstream revision `7dea77e`),
and nothing at runtime depends on the sibling checkouts.

## The open problem: end-effector following (READ THIS FIRST)

The operator reports the arm does not follow sustained tracker translation.
This is the active investigation; the input signal is the prime suspect, not
the control stack.

### Evidence chain, in order

1. Offline, the control stack was progressively exonerated and improved:
   `translation_scale` was 0.5 (halving all motion) and is now 1.0; IK reaches
   static reachable targets to 0.0-1.5 mm; a null-space posture attractor
   (cost 1.0, measured tradeoff documented in `ik.py`) stopped the elbow
   drifting 0.88 rad per closed loop. All real fixes, none of them the
   reported symptom.
2. A dedicated instrumented recording
   (`teleop_dual_fr3.py --debug-log`, analyzer
   `teleop_sources/pico/scripts/simulation/analyze_follow_log.py`) of a real
   right-arm session at
   `/home/descfly/franka_teleop_data/follow_debug.jsonl` showed: the right
   tracker's POSITION took 37 distinct values in 45 s (sub-hertz, frozen for
   5 s stretches, re-locking in jumps up to 119 mm) while its ROTATION moved
   on 84% of ticks. The mapper error was zero, IK lag under a millimetre,
   command-versus-measured small: the arm faithfully tracked a broken input.
3. The operator then isolated the trigger by watching the vendor Unity demo:
   with the tracker in the headset's view its position updates accurately;
   when the wrist flips into a side-grasp pose the tracker position freezes
   and then jumps violently, WHILE optical hand tracking of the same hand
   keeps updating accurately. Position comes from the headset cameras seeing
   the tracker; rotation comes from the tracker's own IMU; they fail
   separately.
4. A second, possibly compounding cause is documented in this repo and was
   live at the time: the desktop `RobotLinuxDemo` GUI and the Python SDK
   compete for the PC Service feedback stream. The GUI was running on the
   machine during the investigation. `create_pico_input` now refuses to start
   while the GUI runs (as the diagnostic scripts always did), so this cannot
   contaminate future sessions, but it may have contaminated the recording in
   point 2.

### What is proven versus assumed

Proven: our client received sub-hertz positions with live rotations; the
control stack downstream of the input is accurate; optical hand tracking
survived the exact poses that killed the tracker position; the frozen-pose
safety guard was blind to this failure mode because rotation kept refreshing
its single liveness clock (now split: position and rotation are judged
independently, either frozen for >1 s disengages, tests lock both directions).

Assumed, NOT yet proven: that headset-camera occlusion during wrist flips is
the mechanism (versus GUI competition alone, versus something else such as a
service-side mode or firmware behaviour). The operator explicitly asked for
this to be established before any redesign.

### The next task: a controlled tracker-visibility experiment

Goal: determine whether the tracker position stream genuinely degrades when
the tracker leaves the headset's view, with the GUI closed, and quantify it.

Suggested protocol (no robot needed; hands/arms stay off):

- Preconditions: RoboticsService and the headset app running, Object motion
  tracking mode, operator WEARING the headset, desktop GUI CLOSED (the tools
  now refuse otherwise). Right tracker worn as configured
  (right = `PC2310MLL5290914G`, left = `PC2310MLL6020917G`).
- Record with `teleop_sources/pico/scripts/hardware/inspect_hand_tracking.py
  --log PATH --duration 90` (single guarded SDK client; logs `motion_poses`
  at 10 Hz alongside hand skeletons).
- Phases of ~15 s each, with the operator noting rough transition times:
  1. tracker plainly in the headset's view, slow continuous translation;
  2. wrist flipped into the side-grasp pose that reproduced the failure,
     still translating;
  3. tracker deliberately hidden (behind the body or under the desk), moving;
  4. back in plain view.
- Analysis: per phase, the distinct-position count per second, per-sample
  displacement distribution, and jump sizes on re-lock. The session scripts
  that did this for the existing recordings were scratch files; the analysis
  is a few lines (load JSONL, extract `motion_poses` by serial, diff). Compare
  against hand-skeleton wrist (joint index 1) liveness in the same phases.
- Optionally repeat the same protocol once with the GUI deliberately open
  (using the raw probe, not teleop) to separate GUI competition from
  occlusion in a 2x2.

Decision tree after the experiment:

- Occlusion confirmed: prefer physical mitigations first (tracker placement/
  orientation on the wrist so its LED ring faces the headset in grasp poses),
  then operational ones (keep the wrist visible; the new frozen-position
  disengage at least fails safely). The operator has REJECTED, for now, both
  pure hand-root EE control (wrist position noise measured at 10-22 mm std
  with rare ~0.9 m skeleton teleports, versus sub-mm tracker when live) and a
  tracker-IMU-rotation + hand-root-translation fusion (judged too fragile).
  Do not build either without explicit new buy-in.
- Occlusion NOT confirmed (stream fine out of view): the recording in point 2
  was likely GUI contamination; the guards already in place should suffice;
  re-run a `--debug-log` teleop session to confirm following is restored.
- After the input is trustworthy, revisit speed: offline replay showed the
  0.5 rad/s joint-speed clamp saturating on 97.5% of ticks and costing 18-47%
  of amplitude at natural hand speeds (hardware limits are 2.62-5.26 rad/s).
  Raising it means changing BOTH `config/pico.yaml` `host.max_joint_speed`
  and `config/teleop_control.yaml` `max_joint_speed` (the gateway clamps,
  never rejects). This was deliberately deferred so it could not mask the
  input problem.

## Facts that were expensive to learn (beyond the runbook)

- One process may own one XRoboToolkit SDK client. Hand tracking and Object
  Motion Tracking coexist in a single client (verified: motion timestamps
  statistically identical with hands active or not); two Python clients have
  never been shown safe, and the desktop GUI counts as a competing client.
- `isActive` is load-bearing for hand skeletons: complete, plausible pose
  arrays keep being served after tracking loss. Liveness comes from
  change-detection; a still hand still jitters (zero bitwise-identical
  consecutive frames in 45 s), so exact constancy means a frozen cache. The
  shared implementation is `SkeletonLiveness` in `hand_input.py`; the tracker
  equivalent now keeps separate position and rotation clocks in
  `xr_input.py`.
- The ROS joint-state topics list joints in broadcaster order, NOT sorted
  (`name: [joint1, joint3, joint5, joint2, ...]`); anything parsing them must
  map by name. Related trap: stripping YAML list items with
  `.strip('- \\n')` also eats the minus sign of negative numbers; this
  produced a false sign-flip alarm during pose capture.
- The operator process must not see a sourced ROS environment (distro
  pinocchio shadows the Conda one via PYTHONPATH, distro libeigenpy breaks it
  via LD_LIBRARY_PATH, which cannot be repaired in-process). `env_guard.py`
  scrubs and re-execs; all host entry scripts call it.
- Vendor driver quirks are listed in HARDWARE_DEPLOY.md (100 Hz silent
  command drop, first state message malformed, no G20 speed initialization,
  python-can>=4 needed, `--symlink-install` required, `/cb_hand_setting_cmd`
  applies to every driver instance regardless of the hand_type field).
- Retargeting facts: OpenXR `*_proximal` (not `*_metacarpal`) is the MANO
  knuckle analogue (3.6x scale trap); human/robot correspond at the
  finger-base centroid scaled by palm width (the L20 URDF base origin sits
  ~100 mm proximal of its knuckles); per-finger chain-length normalization
  prevents finger fouling; abduction takes ONE sign per side
  (`ABDUCTION_INVERTED = {left: False, right: True}`, derived from URDF
  geometry plus one hardware observation, OPPOSITE to the vendor's own
  direction table, locked by `test_abduction_polarity_matches_urdf_geometry`).
- Remaining known hand limits: abduction clips against the URDF's narrow
  +/-0.17 rad; its neutral is not centred per finger; the left thumb solver
  pins `thumb_ip`; retargeting is position-only with no collision model.
- The right-arm initial pose was re-captured by the operator on 2026-07-25
  (`initial_pose.yaml` + MuJoCo `home` keyframe carry identical digit
  strings; a test enforces the match at 1e-12).
- If the Linker hands go silent on CAN, replug the USB-CAN adapters first;
  clean error counters prove nothing (pcan_usb reports TX complete without
  bus acknowledgment). Discovery: `cansend canX 0FF#C0` with a listener
  already running; hands identify themselves and their side in ASCII.

## Data assets on the workstation

- `/home/descfly/franka_teleop_data/hand_coexistence.jsonl` - 45 s, clean,
  full-rate: hand skeletons AND tracker poses at 10 Hz. Ground truth for
  coexistence, scale calibration, and the wrist-versus-tracker comparison.
- `/home/descfly/franka_teleop_data/follow_debug.jsonl` - 44.9 s instrumented
  teleop session with the degraded tracker input (the evidence in point 2).

## Shelved / rejected (do not resurrect without the operator)

- Data collection (Orbbec camera node, recording hand topics, consolidating
  the CLI so the PICO keyboard drives record/save/discard): designed but
  shelved by the operator; the existing `teleop_data` stack records arms only.
  The Orbbec camera is currently not plugged in.
- Raising `max_joint_speed`: deferred until the input is trustworthy (above).
- Hand-root or fusion EE control: rejected pending the visibility experiment.
- C++ rewrite of the operator process: rejected; the 5x retargeting win was
  algorithmic (pinocchio + per-finger solves), and Python is not the
  bottleneck.
- ROS Jazzy migration of the container: rejected as not worth it; the
  Humble/Jazzy split dissolved anyway when the hand stack moved into the
  container.

## Session-tooling notes for the next agent

The workstation is reached through the `agent-remote` MCP workspace named
`bimanual` (root `/home/descfly/hsc`, physical paths under `/home/descfly`).
Quirks observed this session: numeric parameters of `read_file`
(`offset`/`limit`) and `run_command` (`timeout_ms`) fail input validation, so
page files with `sed -n` and keep commands under ~50 s (long builds:
`setsid nohup ... > $AGENT_REMOTE_SCRATCH/log 2>&1 &` then poll). `pkill`
patterns match your own `bash -lc` command line; use `[b]racketed` patterns.
The conda env is `franka-teleop-pico`; run tests and scripts through
`conda run --no-capture-output -n franka-teleop-pico ...`. Do not start
`linker_hand_advanced_g20` (it snaps the hand to a pose at full speed on
construction); `linker_hand_sdk` with `hand_joint:=G20` is motionless. Never
move the arms without the operator present, and disengage PICO before any
`/reset`.

# Contact tolerance and IK transparency: audit and validation plan

Offline audit of research agenda items 1 and 2, 2026-07-29. All hardware
steps below are operator-run; nothing here moves the arms by itself.

## Findings, with evidence

### Item 1: why table contact red-lights the arm

1. `set_bi_collision_behavior.py` never ran: no launch file, compose
   service, or runbook step invokes it (repo-wide grep), so both arms run
   on the controller's default reflex thresholds, not the raised values in
   the script.
2. The script was doubly broken had it run: `SetFullCollisionBehavior.srv`
   uses fixed-size arrays, so the four unset acceleration fields went out
   as zeros (instant reflex on any accelerating motion), and the response
   was never checked - franka_ros2 reports libfranka rejections in
   `response.success` (`franka_param_service_server.hpp`,
   `setGenericRobotParam`), not as call failures. Both fixed.
3. The structural cause is unbounded torque growth, independent of
   thresholds: the impedance controller applies
   `tau = k_gains * (q_goal - q) - d_gains * dq` (k = 600/600/600/600/
   250/150/50) clamped only at hardware torque limits, and the gateway
   slew limit lets `q_goal` keep walking into an obstacle at 0.5 rad/s.
   Sustained contact therefore always ends in a reflex; thresholds only
   set how soon.
4. No force-at-reflex measurement exists offline: the 200 Hz
   `franka_robot_state_broadcaster` runs but none of its topics are in
   `recording.yaml`, and no franka-control container logs survive.

Prepared changes (2026-07-29, second pass):

- Setter script fixed (all eight fields, response verified, values logged)
  and run automatically at bringup as a one-shot node in
  `robot_control.launch.py` - the same pattern Franka's own
  `franka_ros2_teleop` example uses, which calls the service from launch.
- Contact torque gating in the safety gateway, always on: the gateway
  subscribes to each arm's
  `franka_robot_state_broadcaster/external_joint_torques`
  (`tau_ext_hat_filtered`, the same signal Franka's teleop example feeds
  back to its leader arm) and holds any joint whose external torque
  exceeds its threshold from stepping in the loading direction - the sign
  test needs no kinematics because tau_ext is already Jacobian-mapped.
  The unloading direction always passes, so retreat releases instantly. A
  sustained press settles near the threshold (a few Nm) instead of
  winding up to the reflex. Missing or stale torque data fails open with
  a throttled warning; the reflex thresholds below still protect.
- An earlier optional `max_command_deviation` cap was removed in favor of
  the gating: one contact mechanism, no off-by-default safety flags. Its
  protection band (~36 Nm) overlapped what the reflex thresholds already
  provide. (Recoverable from git history if the gating trial fails.)

Threshold layering, all in torque units: gating thresholds (3-6 Nm,
normal contact) < Franka reflex (35-60 Nm, faults) < controller clamp at
the FR3 maxima (87/12 Nm) < firmware current and thermal protection.

### Item 2: unreachable pose or IK failure

The IK degrades through three silent mechanisms: the joint-speed clamp,
position-limit clipping, and QP frame-task residual at workspace edges.
`ik.py` now records per-step diagnostics (position/orientation residual,
saturated joints, near-limit joints, smoothed error progress) and
`classify_step` names the binding constraint: `ok`, `joint-limit`,
`speed-clamp`, `workspace`. A measured subtlety drove the design: at a
workspace edge the QP chatters at the velocity clamp with zero net
progress (1.5 m target settles at 1.11 m error, clamp saturated every
tick), so saturation alone misreads stuck as catching-up; the progress
term separates them.

Offline classification of all 14 replayable sessions (extended
`analyze_follow_log.py`, same rule as the live classifier):

- 20260727_122704 (hand-roots): 3.6% of the engaged segment in deficit,
  138 ticks pinned at j7's limit (9.9% of all ticks within 0.05 rad),
  plus j1 and j3 hits - joint limits were a real, invisible failure mode.
- 20260727_135507: 8.0% and 7.4% of two segments in deficit, dominated by
  workspace-edge ticks; speed clamp saturated on 10.2% of ticks.
- 20260726 tracker sessions: 0.6-2.2% clamp saturation, occasional 1%
  deficit segments, j2/j6 limit grazes in 20260726_212245.
- Quiet sessions (20260726_201920, 20260729 mixed): no deficit ticks.

Live surfacing: the once-per-second STATE line gains
`ik: right LIMIT j7 off 32mm/5deg`-style clauses showing each side's
worst tick since the last report; the debug log (schema follow-debug.v3)
carries the same facts per tick in an `ik` record.

## Operator validation plan

Contact protection is hand-aware by policy: until the LinkerHand vendor
force ratings are known, contact trials touch through the palm, wrist, or
a pad - never fingertips first. The Franka wrench estimate cannot tell a
fingertip from the palm.

Phase A - verify bringup and calibrate the gating thresholds:

1. Bring up the stack. Verify both `accepted the collision thresholds`
   lines in the franka-control log, and the gateway's
   `Contact torque gating active` line in teleop-control. If an arm
   answers `command exception error`, the running control loop refused
   the thresholds: note the timing and re-run the setter through the
   tools container before the first engagement.
2. Sign probe (mandatory before the first contact trial; 2026-07-29: a
   blocked reaching arm hit `cartesian_reflex` with gating active, and an
   inverted tau_ext sign convention is one of the two candidate causes).
   With both arms up and teleop disengaged, record while pushing and
   pulling each arm gently in varied directions - hands off for the
   first five seconds:

   ```bash
   cd /home/descfly/hsc/franka_upper_body_teleop/docker
   docker compose run --rm tools python3 \
     /workspace/franka_upper_body_teleop/scripts/record_tau_ext_probe.py \
     --output /data/diagnostics/tau_probe_$(date +%Y%m%d_%H%M%S).jsonl \
     --duration 60
   ```

3. `python3 scripts/analyze_tau_ext_probe.py <recording>` decides the
   sign convention mechanically (ground truth is the impedance spring:
   a push deflects each joint in the push direction, so sign(dq) labels
   every sample) and prints per-joint quiet noise plus a suggested
   threshold. MATCH: proceed. INVERTED: flip the gating comparison in
   `teleop_core/safety.py` first. Until this verdict exists, treat the
   gating as absent.
4. Set `contact_torque_thresholds` from the suggestions (shipped
   placeholders are `[6, 6, 6, 6, 3, 3, 3]` Nm), then a free-space
   regression: 2-3 min of normal teleop including fast sweeps must feel
   unchanged, with no torque warnings in the gateway log. If fast sweeps
   graze the thresholds, raise to the analyzer's suggestion times two.

Phase B - contact trial (one side engaged, pad on the table):

5. Descend slowly onto the pad, keep pushing the tracker downward 2-3 s,
   retreat. Expect: the arm stops at the surface with a bounded push
   (roughly the threshold torque), no reflex, and immediate release on
   retreat. Keep the bag recording running to read the settled
   `tau_ext` plateau.
6. If a reflex still fires, save `docker compose logs franka-control`
   before `down` and read the wrench at the reflex from the bag; that
   number decides between raising reflex thresholds further and lowering
   the gating thresholds.
7. Threshold philosophy for any adjustment, from Franka's own teleop
   example: proximal reflex ceilings high (they run 85 Nm), wrist
   thresholds tight (they run 11 Nm on joints 5-7) - the wrist is what
   the dexterous hand hangs from.

IK transparency check (any session): stage a full-arm stretch (expect
`UNREACHABLE`), a wrist roll to the j7 stop (expect `LIMIT j7` - the
dominant historical event), and a fast sweep (expect a transient
`clamped`); confirm the STATE line and the v3 log agree.

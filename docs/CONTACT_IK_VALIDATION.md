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

Prepared changes: fixed setter script (all eight fields, response
verified, values logged); optional per-joint `max_command_deviation` cap
in the safety gateway, default `[0.0]` = off, config comment carries the
candidate values. Cap sizing evidence from the 2026-07-26 tracker session
(engaged right arm, 67 s): per-joint |cmd - meas| p99
0.034/0.018/0.021/0.021/0.021/0.078/0.056 rad, max
0.045/0.040/0.035/0.037/0.040/0.090/0.111 rad. Candidate
`[0.06, 0.06, 0.06, 0.06, 0.12, 0.2, 0.3]` sits above every measured
maximum and bounds static contact torque at <= 36 Nm on every joint.

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

Phase A - thresholds (config only, no behavior change in free space):

1. Bring up the stack; then `docker compose run --rm tools ros2 run
   franka_fr3_arm_controllers set_bi_collision_behavior.py`. Expect both
   "accepted" lines. If an arm answers "command exception error", the
   running control loop refused it: retry immediately after
   `franka-control` starts, before the first engagement, and note which
   ordering worked in this file.
2. Find the state topic (`ros2 topic list | grep robot_state`) and record
   it during the trial with `ros2 bag record` into the session RUN_DIR.
3. Controlled contact, one side engaged, compliant pad on the table:
   descend slowly until touch, hold 2 s, retreat. If a reflex still
   fires, read `K_F_ext_hat_K`/`O_F_ext_hat_K` at the reflex from the bag
   and save `docker compose logs franka-control` before `down`.
4. Success: a light touch no longer red-lights; the measured force at any
   remaining reflex tells whether to move thresholds or go to Phase B.

Phase B - deviation cap (one flag, revert with `[0.0]`):

5. Set the candidate cap in `config/teleop_control.yaml`; free-space-only
   teleop for 2-3 min with `--debug-log`; replay `analyze_follow_log.py`
   and require per-joint engaged p99 below each cap and no felt change.
6. Repeat the contact trial: expect bounded push force, no reflex, and
   immediate release when the operator retreats.

IK transparency check (any session): stage a full-arm stretch (expect
`UNREACHABLE`), a wrist roll to the j7 stop (expect `LIMIT j7` - the
dominant historical event), and a fast sweep (expect a transient
`clamped`); confirm the STATE line and the v3 log agree.

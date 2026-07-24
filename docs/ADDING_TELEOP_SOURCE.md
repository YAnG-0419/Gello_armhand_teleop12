# Adding a teleoperation source

A source adapter must publish `teleop_interfaces/msg/ArmCommand` on
`/teleop/arm_commands`. It must never publish
`/target_robot/joint_commands`.

Each process lifetime uses a new non-empty `session_id`. Sequence numbers
start at zero and increase strictly. `active_sides` determines the exact
canonical joint-name sequence:

- `left`: `left_fr3v2_joint1` through `left_fr3v2_joint7`
- `right`: `right_fr3v2_joint1` through `right_fr3v2_joint7`
- `left, right`: the left sequence followed by the right sequence

An adapter can obtain joint targets in different ways:

- PICO or an exoskeleton can retarget end-effector poses through IK.
- GELLO can map leader joints directly.
- Replay reads recorded joint targets.

The safety gateway arbitrates source ownership, reacquires every new session,
checks fresh measured state, validates the first target, applies FR3 limits
and slew limits, and publishes the hardware command only when explicitly
enabled.

Add the new source name to the gateway's `allowed_sources` parameter only
after its adapter tests cover session restart, stale input, malformed joints,
and disengagement.

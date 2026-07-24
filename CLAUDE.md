# Repository rules

- Existing sibling repositories are read-only references. Copy required code into this repository before changing it.
- `teleop_core.contract` is the single source of truth for robot topics and joint names.
- Every teleoperation source publishes `teleop_interfaces/ArmCommand`; only the safety gateway may publish the FR3 command bus.
- Hardware output defaults to disabled. Never weaken freshness, acquisition, limit, or slew checks to make a test pass.
- Keep source adapters, safety/control, and data tooling independent.
- Do not add glue-dispenser terminology or functionality.

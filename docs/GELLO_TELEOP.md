# Dual GELLO incremental teleoperation

This first stage changes only the arm input. The inherited workcell remains:

- left/right FR3 controlled by the existing ROS 2 impedance controllers;
- existing UDP bridge and safety gateway;
- left G20 and right O30i;
- MANUS hand input and independent hand worker.

No new dataset recorder is enabled in this stage. `JointTeleopSample`, mapper
diagnostics, and `debug_feed_state()` retain timestamps and calibrated leader
joints so a recorder can be added later without changing the control contract.

## Incremental mapping

Each side anchors independently on its GUI engage edge:

```text
raw_delta = calibrated_gello_now - gello_at_engage
scaled_delta = raw_delta * joint_sensitivity
target = robot_at_engage + clip(scaled_delta, -max_relative_delta, max_relative_delta)
```

Disengaging and re-engaging captures fresh anchors, so an absolute GELLO/FR3
pose match is unnecessary and the first command equals measured robot state.
Each side has seven positive `joint_sensitivity` values in `config/gello.yaml`,
ordered by GELLO motor IDs 1-7. A value of `2.0` maps one degree of calibrated
GELLO displacement to two degrees of FR3 target displacement; `0.5` provides
half-scale fine control. Direction remains exclusively controlled by
`standard_signs` and `direction_correction`. Sensitivity is restricted to
0.1-2.0 and is applied before the relative-displacement limit.

The operational `max_relative_delta` is 1.5 rad per joint and GELLO targets
slew at 0.5 rad/s. The unchanged ROS safety gateway additionally enforces FR3
joint limits, first-target distance, its 0.5 rad/s outer slew ceiling, command
freshness, reset exclusion, and contact-torque gating.

GELLO motor 8 is never opened. MANUS exclusively owns both dexterous hands.

## Verified historical identities

`config/gello.yaml` contains the previous hardware mapping:

- left: `FTATCZ4W`
- right: `FTALZ24C`

At the 2026-08-01 check, both identities were enumerated: `FTATCZ4W` as
`ttyUSB1` and `FTALZ24C` as `ttyUSB0`. The current login session had not yet
picked up its `dialout` membership, although `/etc/group` was correct. Never
infer left/right from `ttyUSB0` or `ttyUSB1`.

Run the read-only check after reconnecting both units:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python scripts/check_gello_ports.py --config config/gello.yaml
```

If permission fails, add the operator to `dialout`, log out completely, and
log in again. Do not weaken the preflight or replace by-id paths with ttyUSB
paths.

## Driver setup

The dedicated environment is `gello-upper-body-teleop`. To reproduce the
external driver installation, create or reuse a `gello_software` checkout:

```bash
git clone https://github.com/wuphilipp/gello_software.git /path/to/gello_software
git -C /path/to/gello_software submodule update --init third_party/DynamixelSDK
GELLO_SOFTWARE_ROOT=/path/to/gello_software scripts/setup_gello_driver.sh
```

The runtime uses `gello.dynamixel.driver.DynamixelDriver` directly with IDs
1-7 at 57600 baud. It does not use GELLO's gripper configuration.

Before starting either Franka, hold both GELLOs still and inspect their live
joint streams:

```bash
sg dialout -c 'conda run --no-capture-output -n gello-upper-body-teleop python scripts/inspect_gello_joints.py --duration 5'
```

The upstream driver writes torque-disable once during initialization, then
reads present position and velocity. This diagnostic never enables torque,
sends a goal position/current, or connects to either Franka. The serial IDs,
joint ordering, standard signs, and per-arm direction corrections are reused
from `/home/descfly/llx/gello_franka`; no recalibration is required while the
hardware and assembly remain unchanged.

## Start without recording

After the ordinary Docker build and GELLO preflight:

```bash
cd docker
docker compose up franka-control teleop-control gello-bridge hand-control
```

In another terminal:

```bash
scripts/run_teleop.sh
```

Then start the existing operator GUI. The backend starts disengaged.

For first hardware motion:

1. Keep the emergency stop reachable and clear both workspaces.
2. Hold both GELLO units still.
3. Engage only one side.
4. Make one small known motion as a smoke test of the reused mapping.
5. Disengage before moving the leader to another comfortable pose.
6. Smoke-test the other side, then test both sides.

Any stale sample, serial error, gateway rejection, robot-state loss, GUI loss,
or reset disengages the affected path. Re-engage always creates a new anchor.

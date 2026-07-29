# MANUS right-hand teleoperation

Powered by Manus.

The primary operator path uses the calibrated 25-keypoint MANUS skeleton,
retargets it against the right O30i URDF, and sends a named 20-joint radians
packet to `linker_hand_bridge`. The left G20 receives its all-zero L20 default.
PICO optical hand tracking is not used for O30i.

The older standalone C++ ergonomics-angle adapter remains available for the
dual-G20 setup. It does not support O30i.

Only the right-glove path is implemented. Bimanual MANUS is future work.

## Build

Only CMake, a C++17 compiler, pthreads, and the system libraries needed by the
vendored MANUS SDK are required:

```bash
teleop_sources/manus/scripts/build.sh
```

## Dry run

Do not run this while another MANUS CoreSDK client owns the glove. Dry run
connects to MANUS but does not send UDP packets:

```bash
teleop_sources/manus/build/manus_right_hand_teleop --duration 10
```

It prints raw right-hand degrees and the resulting robot qpos. Use this first
to verify that an open hand is near zero flexion and closing each finger makes
the corresponding positive angles increase. By default the adapter applies
`config/Calibration_right.mcal`, copied from the supplied Metaglove Pro
reference to match the detected `MetagloveProHaptics`, after it discovers the
right glove and before permitting output. Use
`--calibration FILE` to select another saved calibration or `--no-calibration`
to retain MANUS Core's current calibration.

For the older standalone adapter, `--send` is required to transmit packets.
It stops sending both sides when no fresh right glove frame has arrived for
250 ms; the bridge watchdog then holds the hands. The bridge's own
hardware-enable setting remains the final physical-output gate.

Useful options:

```text
--host 127.0.0.1
--port 5570
--rate 30
--stale-timeout 0.25
--duration 0
--network-discovery
--calibration config/Calibration_right.mcal
--no-calibration
```

The default discovery scope is localhost, matching the installed MANUS
Robotics Service. Use `--network-discovery` only when MANUS Core runs on
another host.

## O30i robot-side dry run

This validates model tags, joint order, limits, and retargeting without opening
the O30i CAN-FD device:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
ros2 launch linker_hand_bridge hands.launch.py \
  sides:=both left_model:=g20 right_model:=o30i enabled:=false
```

The real-test wrapper uses the vendor's normalized full-range mapping by
default: tick 0 at each URDF lower limit and tick 255 at each upper limit.
Supplying `O30_TICKS_AT_LOWER` and `O30_TICKS_AT_UPPER` overrides it with
per-device endpoints. The O30i driver does not enable its motors before the
first complete valid radians command. A 250 ms command timeout or 500 ms
valid-feedback timeout disables every O30i joint terminally and requires a
node restart.

The O30i retargeter contains no operator-specific or per-finger PIP/DIP gain.
Any future operator calibration should be an explicit profile derived from a
multi-pose recording rather than an embedded correction.

## Primary O30i operator

Run arm and hand teleoperation in one operator process. This is the primary
command; it requires only the right motion tracker and uses the same `R`,
`Space`, and `X` state to gate both the right arm and right hand. For this
source, `--right-hand-model` defaults to `o30i`:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers \
  --hand-source right-only-manus
```

The standalone `scripts/teleop_full_thumb.py` remains a G20 hand-only
diagnostic, not the O30i path.

For an O30i hand-only test, use `scripts/teleop_o30i.py`. It starts disengaged
and requires `Space` or `R` before it sends right-hand packets. See
`docs/HARDWARE_DEPLOY.md` for the dry-run and real robot-side commands.

The normal real-hardware entry points are `scripts/run_o30_robot.sh` in the
robot terminal and `scripts/run_o30_manus.sh` in the MANUS terminal.
The hand-only path has been physically validated. The integrated
PICO-motion-tracker plus mixed left-G20/right-O30i run remains pending.

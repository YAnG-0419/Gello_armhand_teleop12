# MANUS right-hand teleoperation MVP

Powered by Manus.

This adapter reads the right glove's 20 MANUS ergonomics angles and sends the
repository's existing 21-joint L20 UDP packet to `linker_hand_bridge`. It is a
standalone, ROS-free C++ process. The left side sends the all-zero default pose
while right-hand input is live.

The adapter maps each MANUS spread/MCP/PIP/DIP angle (degrees) to the
corresponding L20 coordinate (radians), clipped to the URDF limits. The right
thumb retains the hardware-validated fixed opposition `(yaw=1.10, roll=0.52)`;
MANUS thumb stretches drive pitch, MCP, and distal flexion. The existing bridge
still owns G20 projection, slew limiting, freshness, and hardware enablement.

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

## Send to the hand bridge

Start only the hand stack, then explicitly enable adapter sending:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up -d hand-control

cd /home/descfly/hsc/franka_upper_body_teleop
teleop_sources/manus/build/manus_right_hand_teleop --send
```

The `hand-control` service defaults are ready for the current MANUS MVP:
both G20s, output enabled, the hardware-validated finger-spread polarity, 1500
unit/s slew, speed 255, finger torque 200, and thumb torque 250. The
`right-only-manus` source continuously sends the default pose to the left hand.
No launch arguments are required. The generic low-level bridge remains
output-disabled by default for diagnostics.

`--send` is required. The adapter stops sending both sides when no fresh right
glove frame has arrived for 250 ms; the bridge watchdog then holds the hands.
The bridge's own hardware-enable setting remains the final physical-output
gate.

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

## Experimental full-thumb retargeting

`scripts/teleop_full_thumb.py` uses the calibrated 25-node MANUS raw skeleton,
converts it to the existing canonical 21 landmarks, and runs the established
L20 solver with `thumb_opposition_fixed=None`. This frees thumb yaw, roll,
pitch, and coupled distal flexion instead of using the fixed opposition from
the ergonomics MVP.

The normal `hand-control` defaults include the right G20 abduction correction.
Hardware testing confirmed the previous polarity closed the finger gaps when
the operator spread them. Start the robot-side services from the Compose
directory:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up hand-control franka-control teleop-control pico-bridge
```

Run arm and hand teleoperation in one operator process. This is the primary
full-thumb command; it requires only the right motion tracker and uses the same
`R`, `Space`, and `X` state to gate both the right arm and right hand:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers \
  --hand-source right-only-manus
```

The standalone `scripts/teleop_full_thumb.py` remains a hand-only diagnostic,
not the full teleoperation entrypoint. Omitting its `--send` connects, solves,
and prints diagnostics without emitting UDP commands. The full-thumb mode is
experimental: in the first hardware test all thumb coordinates responded
across their available ranges, but its tip error reached roughly 33 mm in deep
opposition, so operator feel still needs comparison against fixed opposition.

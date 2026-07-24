# Dual-FR3 deployment

Current workcell:

- left FR3: `172.16.0.3`
- right FR3: `172.16.0.2`
- host interface: `enp6s0` (`172.16.0.6/24`)
- ROS domain: `0`

Keep the emergency stop reachable. Keyboard activation always starts with both
arms disabled.

## One-time setup

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
cp docker/.env.example docker/.env
./scripts/build.sh
./scripts/setup_pico_env.sh
```

Review all four entries in `docker/.env`, especially
`FRANKA_ROBOT_CONFIG`. Runtime shell variables do not override this file.
Robot addresses come only from that selected workcell YAML; PICO networking
and motion scale come only from `config/pico.yaml`. Startup stops with an
error if required configuration is absent or misspelled.

Before each session, activate both arms and FCI in Franka Desk, then check:

```bash
ping -c 2 172.16.0.3
ping -c 2 172.16.0.2
```

## Run

Start XRoboToolkit PC Service, connect the headset, and enable motion-tracker
streaming. On the first setup, identify the trackers:

```bash
./scripts/list_pico_trackers.sh
```

Move one tracker at a time, note its serial, then set `input.serials.left` and
`input.serials.right` in `config/pico.yaml`.

The tracker-to-control transforms initially use identity. Keep them unchanged
for the first dry run. If rotating a tracker causes unwanted translation
because it is mounted away from the intended wrist/control point, measure that
fixed offset and enter it under `input.tracker_to_control`.

Before starting either robot, verify live tracker motion and keyboard
activation in MuJoCo:

```bash
./scripts/run_simulation.sh
```

Terminal 1:

```bash
./scripts/start_franka.sh
```

Terminal 2:

```bash
./scripts/start_pico_enabled.sh
```

Terminal 3:

```bash
./scripts/run_pico.sh
```

For a session without robot output, use `./scripts/start_pico_dry_run.sh` in
Terminal 2 instead.

The PICO terminal starts with both arms disabled:

- `Space`: toggle both arms
- `L` / `R`: toggle one arm
- `X`: immediately disable both arms
- `Q`: disable both arms and exit

Enabling an arm anchors its tracker to the current end-effector pose. Disabling
allows the tracker to be repositioned without moving the robot. Missing or
stale tracker data disables both arms and requires deliberate re-enabling.

The existing PICO coordinate conversion expects:

- tracker forward → `+X`
- tracker right → `-Y`
- tracker up → `+Z`

Confirm those directions in MuJoCo because the motion-tracker stream could
differ from the former controller stream. On the first robot run, engage one
arm at a time and use a small translation before testing rotation or bimanual
motion.

## Operator, recording, and reset

Open another terminal:

```bash
./scripts/operator.sh
```

Useful commands are:

- `/capture`: read both measured joint-state topics and save them as the reset pose
- `/reset`: smoothly return both arms to the saved pose
- `/record`, `/stop`, `/save`, `/discard`: manage an episode
- `/status`: show joint-state and reset-service availability

`/capture` does not publish a robot command. Before `/reset`, press `X` in the
PICO terminal. The reset temporarily suppresses teleoperation commands while
it interpolates both arms at `0.15 rad/s`.

Stop `run_pico.sh` before replay:

```bash
./scripts/convert.sh /data/episodes/episode0 /data/normalized/episode0.npz
./scripts/replay.sh /data/normalized/episode0.npz
```

## Shutdown

Press `X`, stop Terminal 3, stop Terminal 2, then stop Terminal 1. Disable FCI
in Franka Desk when the workcell is unattended.

# Dual-FR3 deployment

Current workcell:

- left FR3: `172.16.0.3`
- right FR3: `172.16.0.2`
- host interface: `enp6s0` (`172.16.0.6/24`)
- ROS domain: `0`

Keep the emergency stop reachable and begin with both controller grips
released.

## One-time setup

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
cp docker/.env.example docker/.env
./scripts/build.sh
./scripts/setup_pico_env.sh
```

Before each session, activate both arms and FCI in Franka Desk, then check:

```bash
ping -c 2 172.16.0.3
ping -c 2 172.16.0.2
```

## Run

Start XRoboToolkit PC Service and connect the headset.

Terminal 1:

```bash
ROS_DOMAIN_ID=0 ./scripts/start_franka.sh
```

Terminal 2:

```bash
ROS_DOMAIN_ID=0 ./scripts/start_pico_enabled.sh
```

Terminal 3:

```bash
./scripts/run_pico.sh
```

For a session without robot output, use `./scripts/start_pico_dry_run.sh` in
Terminal 2 instead.

Squeezing a grip anchors that controller to the current end-effector pose.
PICO motion maps to robot world as follows:

- controller forward → `+X`
- controller right → `-Y`
- controller up → `+Z`

On the first run after a mapping change, engage one arm at a time and use a
small translation before testing rotation or bimanual motion.

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

`/capture` does not publish a robot command. Before `/reset`, release both
PICO grips. The reset temporarily suppresses teleoperation commands while it
interpolates both arms at `0.15 rad/s`.

Stop `run_pico.sh` before replay:

```bash
./scripts/convert.sh /data/episodes/episode0 /data/normalized/episode0.npz
./scripts/replay.sh /data/normalized/episode0.npz
```

## Shutdown

Release both grips, stop Terminal 3, stop Terminal 2, then stop Terminal 1.
Disable FCI in Franka Desk when the workcell is unattended.

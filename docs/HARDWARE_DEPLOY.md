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
FRANKA_TELEOP_ENABLE=I_UNDERSTAND \
ROS_DOMAIN_ID=0 \
./scripts/start_pico_enabled.sh
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

## Record and replay

While the pipeline is running:

```bash
./scripts/record.sh
record> start
record> stop
record> save
```

Stop `run_pico.sh` before replay:

```bash
./scripts/convert.sh /data/episodes/episode0 /data/normalized/episode0.npz
./scripts/replay.sh /data/normalized/episode0.npz
```

## Shutdown

Release both grips, stop Terminal 3, stop Terminal 2, then stop Terminal 1.
Disable FCI in Franka Desk when the workcell is unattended.

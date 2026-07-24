# Dual-FR3 hardware deployment

This guide targets the current workcell:

- left FR3: `172.16.0.3`
- right FR3: `172.16.0.2`
- host interface: `enp6s0`, normally `172.16.0.6/24`
- one switch shared by both arms
- no Franka Hand; dexterous hands are independent
- ROS domain: `0`

Keep the emergency stop reachable whenever the arms can move.

## 1. One-time setup

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
cp docker/.env.example docker/.env
mkdir -p /home/descfly/franka_teleop_data
./scripts/build.sh
./scripts/setup_pico_env.sh
```

Review these files before hardware use:

- `config/current_workcell.yaml`
- `docker/.env`
- `teleop_sources/pico/assets/dual_fr3/scene.xml`

`FRANKA_CPUSET` must contain both per-arm `controller_cpus` ranges from the
workcell configuration. The default assigns `16-19` to the left controller
and `20-23` to the right controller.

## 2. Offline acceptance

All of these must pass:

```bash
./scripts/run_simulation.sh --headless --mock-xr --duration 2
conda run --name franka-teleop-pico \
  python teleop_sources/pico/scripts/simulation/validate_dual_fr3_dynamics.py
./scripts/start_fake_franka.sh
./scripts/test_offline.sh
./scripts/test_dry_run.sh
```

For `start_fake_franka.sh`, wait until both joint impedance controllers are
reported active, then stop it with Ctrl-C before continuing.

`test_dry_run.sh` injects synthetic FR3 states, sends commands through the
real UDP adapter and safety gateway, verifies the validated command, and
fails if `/target_robot/joint_commands` receives anything.

## 3. Network and Franka Desk

```bash
ip -brief address show enp6s0
ping -c 2 172.16.0.3
ping -c 2 172.16.0.2
```

In Franka Desk for both arms:

1. Release brakes and activate the arm.
2. Enable FCI/external control.
3. Confirm there is no emergency stop or user stop.
4. Confirm the physical workspace is clear.

Do not run a realtime configuration script written for other interface names.
Any IRQ or CPU-affinity configuration must target this machine's actual
`enp6s0` topology.

## 4. Start the FR3 controllers

Terminal 1:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
ROS_DOMAIN_ID=0 ./scripts/start_franka.sh
```

Both `joint_impedance_controller` instances must configure and activate. Stop
here if either arm reports connection, realtime, or communication-constraint
errors.

## 5. Start PICO in dry-run

Start XRoboToolkit PC Service and connect the headset.

Terminal 2:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
ROS_DOMAIN_ID=0 ./scripts/start_pico_dry_run.sh
```

The safety gateway must print `mode=DRY-RUN`.

Terminal 3:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
./scripts/run_pico.sh
```

Keep both grips released initially.

## 6. Inspect dry-run output

In another terminal:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools bash -lc \
  'ros2 topic echo --once /teleop/validated_arm_commands'
```

Test one grip at a time with a very small controller motion. A valid message
must contain the expected canonical joints for the active side.

Verify that the hardware command topic stays silent:

```bash
docker compose run --rm tools bash -lc \
  'timeout 3 ros2 topic echo --once /target_robot/joint_commands'
test $? -eq 124
```

Dry-run acceptance requires:

- fresh state from both FR3 arms
- validated left-arm command
- validated right-arm command
- no hardware command message
- no continuing safety rejection

## 7. Record a dry-run episode

The default config records arm states, source commands, validated commands,
and optional `/camera/color/image_raw` and `/camera/depth/image_raw`.

```bash
./scripts/record.sh
record> start
record> stop
record> save
```

Convert it:

```bash
./scripts/convert.sh \
  /data/episodes/episode0 \
  /data/normalized/episode0.npz
```

Do not mark camera topics required until their drivers and exact ROS types
have been validated on this workcell.

## 8. Enable real output

1. Stop the PICO host process.
2. Stop the dry-run `teleop-control` and `pico-bridge` services.
3. Release both grips.
4. Put the emergency stop in hand.
5. Confirm both arms and attached tooling are clear.

Start enabled mode:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
FRANKA_TELEOP_ENABLE=I_UNDERSTAND \
ROS_DOMAIN_ID=0 \
./scripts/start_pico_enabled.sh
```

The gateway must print `mode=ENABLED`.

Then start the host:

```bash
./scripts/run_pico.sh
```

Validate in this order:

1. Left arm only, 1–2 cm motion.
2. Stop and inspect logs.
3. Right arm only, 1–2 cm motion.
4. Stop and inspect logs.
5. Both arms with small, slow motion.

## 9. Replay

Stop the live PICO host before replay so it releases source ownership. Begin
with the gateway in dry-run mode:

```bash
./scripts/replay.sh /data/normalized/episode0.npz
```

Inspect `/teleop/validated_arm_commands`. Only replay with an enabled gateway
after the episode, initial pose, physical scene, payload, and emergency-stop
procedure have been reviewed.

## 10. Shutdown

Use this order:

1. Release both grips.
2. Stop the PICO host or replay process.
3. Stop `pico-bridge` and `teleop-control`.
4. Stop recording.
5. Stop `franka-control`.
6. Disable FCI in Franka Desk if the workcell is unattended.

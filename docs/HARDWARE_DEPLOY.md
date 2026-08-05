# Hardware runbook

This is the short operational path for dual GELLO, dual FR3, left G20,
right O30i, MANUS input, and the Orbbec camera. See
[GELLO_TELEOP.md](GELLO_TELEOP.md) for GELLO-specific preflight and anchoring.

## Safety

- Keep the physical emergency stop reachable. GUI disengage stops following but is not a power cut.
- Never start a second PICO or MANUS client beside a live operator.
- Do not reconfigure `enp6s0`, run Home, replay a dataset, or send diagnostic hand poses unless you intend to move hardware.
- On every bringup, verify both `accepted the collision thresholds` messages in `franka-control` and `Contact torque gating active` in `teleop-control`.
- After a reflex or unexplained fault, save `docker compose logs franka-control teleop-control` before stopping the stack.

## Standard startup

Terminal 1:

```bash
cd /home/descfly/llx/gello_upper_body_teleop/docker
docker compose up franka-control teleop-control gello-bridge hand-control
```

Terminal 2:

```bash
cd /home/descfly/llx/gello_upper_body_teleop
scripts/run_teleop.sh
```

Terminal 3:

```bash
cd /home/descfly/llx/gello_upper_body_teleop
conda activate base
python teleop_sources/gui/operator_gui.py
```

The backend starts disengaged and writes a fresh diagnostics directory. GUI loss disengages all sides. Engage gates each arm and its hand together. `Home arm` moves only the selected arm; `Open hand` first stops that side following and then opens only the selected hand. Each action supports left, right, or both.

## Stop

Disengage in the GUI, stop the operator, stop Compose, then disable FCI. Do not run another VIVE, PICO, or MANUS diagnostic until the operator has exited.

## Tracker checks

Tracker availability is per-side. A missing tracker refuses that side; loss or a motion fault while engaged disengages both sides.

Before connecting any robot process, the default VIVE read-only connectivity check is:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python teleop_sources/vive/scripts/inspect_vive_trackers.py \
  --config config/vive.yaml --watch
```

For optional PICO motion-tracker arms, replace `vive-bridge` with
`pico-bridge`; never run both because they own the same host UDP ports:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose --profile pico up franka-control teleop-control pico-bridge hand-control
```

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
scripts/run_pico_teleop.sh
```

With teleoperation stopped, PICO trackers can be inspected or assigned with:

```bash
conda run --no-capture-output -n gello-upper-body-teleop python teleop_sources/pico/scripts/hardware/inspect_motion_trackers.py
conda run --no-capture-output -n gello-upper-body-teleop python teleop_sources/pico/scripts/hardware/calibrate_tracker_sides.py --write
```

PICO diagnostics use `follow-debug.v6`: `seq` is accepted Motion callbacks,
`callback_errors` counts rejected SDK fields/frames, `n` is usable trackers in
the last parsed frame, and `age` is time since `seq` advanced.

## Alternate input and hand modes

Use PICO optical hands:

```bash
scripts/run_teleop.sh --hand-source pico
```

Use controllers for arms:

```bash
conda run --no-capture-output -n gello-upper-body-teleop python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py --config config/pico.yaml --arm-source controllers
```

Test MANUS hands without arms:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up hand-control
```

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run --no-capture-output -n gello-upper-body-teleop python teleop_sources/manus/scripts/teleop_manus_hands.py --sides both
```

The hands-only controls are `L`/`R`, `Space`, `X`, `O`, and `Q`. The left G20 uses full L20-URDF retargeting. MANUS gloves require `Calibration_left.mcal` and `Calibration_right.mcal` in `teleop_sources/manus/config`.

## Arm home and hand open

Use the GUI for independent per-side `Home arm` and `Open hand` actions. There is deliberately no combined arm-and-hand home action; request both actions explicitly when both are wanted. The arm service equivalent is:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools ros2 service call /reset_to_initial_pose std_srvs/srv/Trigger '{}'
```

Per-side services are `/reset_to_initial_pose/left` and `/reset_to_initial_pose/right`. `/capture_initial_pose` replaces the saved home. Reset is joint interpolation, not collision planning.

## Camera

The default Compose stack includes Orbbec. For camera-only diagnosis:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose up -d orbbec
docker compose logs -f orbbec
docker compose run --rm tools ros2 topic list | grep '^/camera/'
```

Expected images are `/camera/color/image_raw` and `/camera/depth/image_raw`. Do not run OrbbecViewer while ROS owns the camera; network recovery is in [ORBBEC_CAMERA.md](ORBBEC_CAMERA.md).

## Record

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools ros2 run teleop_data operator \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml \
  --qos /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording_qos.yaml
```

Record only after `/status` reports `arms=2/2 | record_topics=12/12 | reset=ready`. The commands are `/record`, `/stop`, and `/save`; episodes go to `/home/descfly/franka_teleop_data/episodes/episodeN`.

## Export and replay

```bash
./scripts/export_lerobot.sh /data/episodes/episode0 \
  --output /data/lerobot/my_dataset \
  --task "describe the demonstrated task" \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml \
  --fps 10
```

Replay moves both arms and hands:

```bash
cd /home/descfly/hsc/franka_upper_body_teleop/docker
docker compose run --rm tools ros2 run teleop_data replay /data/lerobot/my_dataset \
  --episode-index 0 \
  --config /workspace/franka_upper_body_teleop/ros_ws/src/teleop_data/config/recording.yaml
```

Disengage teleop, inspect the preposition path, and keep the emergency stop reachable.

## Verification

```bash
cd /home/descfly/hsc/franka_upper_body_teleop
conda run -n gello-upper-body-teleop pytest -q teleop_sources/pico/tests
PYTHONPATH=ros_ws/src/teleop_core python3 -m pytest -q ros_ws/src/teleop_core/test/
```

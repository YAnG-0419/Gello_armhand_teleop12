# Dual FR3 MoveIt mode

The `lychee_fr3_description` and `lychee_fr3_moveit_config` ROS 2 packages are
copied from `lychee_barmate` and built with the rest of this workspace.  They
are an optional operating mode: MoveIt trajectory controllers and the normal
Gello safety gateway must never own the arms at the same time.

Rebuild the image/workspace after the first checkout:

```bash
./ops/setup/build.sh
```

Validate planning with fake hardware:

```bash
./ops/run/run_moveit.sh --fake
```

After the usual workcell checks, use both physical FR3 arms:

```bash
./ops/run/run_moveit.sh --real
```

The launcher refuses to run while any Gello/PICO/VIVE arm-control service is
active.  Real mode runs `ops/run/preflight.sh` and uses the workcell's FR3
addresses (`172.16.0.3`, `172.16.0.2`).

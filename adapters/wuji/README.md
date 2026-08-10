# Wuji hand integration

This optional backend vendors the retargeting core and MANUS configurations
from `wuji-retargeting`.  It uses the repository's existing MANUS native bridge
and the existing Operator GUI: each side starts disengaged, follows only while
that side is engaged, stops sending on stale MANUS data, and disables the Wuji
hardware during cleanup.

The vendored source is synchronized to upstream revision `66693c2`. Wuji Hand 2
configs use the correct official `hand2_beta` URDF, MJCF, and meshes in
`models/hand2_beta`; no legacy Hand 2 model is retained.

The original O30i stack is retained unchanged and remains the default path of
`ops/run/start_teleop.sh`.  Wuji uses a separate entry point and does not start
the Linker Hand/O30i container.

The real-hand entry point uses these same `hand2_beta` configs. Retargeted qpos
is reordered from the URDF/Pinocchio order into the compiled MJCF/device order
by joint name before it is sent to the SDK. The resolved permutation is printed
at startup so the active mapping can be checked before engaging either hand.

Install the additional Python dependencies once:

```bash
./ops/setup/setup_wuji_env.sh
```

## Simulation

The simulation entry point never connects to Wuji hardware or publishes a
hardware command. It uses the bundled 2751-frame replay by default.

Open the `hand2_beta` model:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim --side right
```

Run a repeatable check without a viewer:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim \
  --side right --headless --frames 300
```

Use `--side left` for the left hand. To visualize a live MANUS glove instead of
the replay, stop every other MANUS client first, start MANUS Core, then run:

```bash
conda run --no-capture-output -n gello-upper-body-teleop \
  python -m adapters.wuji.sim --side right --input manus
```

The live MANUS mode reads the glove and drives MuJoCo only; it still does not
connect to either O30i or Wuji hand hardware.

For two network Wuji Hand 2 devices, addresses are deliberately explicit so
left/right cannot be selected by network discovery incorrectly:

```bash
./ops/run/start_wuji_teleop.sh \
  --wuji-left-address 192.168.1.111:50001 \
  --wuji-right-address 192.168.1.112:50001
```

One original USB Wuji Hand is also supported:

```bash
./ops/run/start_wuji_teleop.sh \
  --wuji-sides right \
  --wuji-right-model wuji_hand \
  --wuji-right-serial SERIAL
```

Wuji Hand 2 defaults match the imported implementation: `kp=3.0`, `kd=0.1`,
and a per-joint current limit of `1.5 A`. Override them only after hardware
validation with `--wuji-kp`, `--wuji-kd`, and `--wuji-current-limit`.

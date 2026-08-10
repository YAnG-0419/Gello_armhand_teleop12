# Wuji hand integration

This optional backend vendors the retargeting core and MANUS configurations
from `wuji-retargeting`.  It uses the repository's existing MANUS native bridge
and the existing Operator GUI: each side starts disengaged, follows only while
that side is engaged, stops sending on stale MANUS data, and disables the Wuji
hardware during cleanup.

The original O30i stack is retained unchanged and remains the default path of
`scripts/start_teleop.sh`.  Wuji uses a separate entry point and does not start
the Linker Hand/O30i container.

Install the additional Python dependencies once:

```bash
./scripts/setup_wuji_env.sh
```

For two network Wuji Hand 2 devices, addresses are deliberately explicit so
left/right cannot be selected by network discovery incorrectly:

```bash
./scripts/start_wuji_teleop.sh \
  --wuji-left-address 192.168.1.111:50001 \
  --wuji-right-address 192.168.1.112:50001
```

One original USB Wuji Hand is also supported:

```bash
./scripts/start_wuji_teleop.sh \
  --wuji-sides right \
  --wuji-right-model wuji_hand \
  --wuji-right-serial SERIAL
```

Wuji Hand 2 defaults match the imported implementation: `kp=3.0`, `kd=0.1`,
and a per-joint current limit of `1.5 A`. Override them only after hardware
validation with `--wuji-kp`, `--wuji-kd`, and `--wuji-current-limit`.

#!/usr/bin/env bash
# One-command unified teleop: fresh RUN_DIR, both debug logs, TUI.
#
# Exists because the equivalent multi-line paste has now buried three
# recordings: a clipboard missing its final newline leaves the command
# pending, and the next paste glues onto it. A single short command has no
# such failure mode.
#
# Extra arguments pass through to teleop_dual_fr3.py and later flags win,
# so e.g. `scripts/run_teleop.sh --hand-source pico` switches the hand
# source, and `--ui plain` disables the TUI. TELEOP_RUN_DIR overrides the
# run directory.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${TELEOP_RUN_DIR:-/home/descfly/franka_teleop_data/diagnostics/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_DIR"
echo "RUN_DIR=$RUN_DIR"

cd "$REPO_ROOT"
exec conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers \
  --hand-source right-only-manus \
  --debug-log "$RUN_DIR/ee_jitter.jsonl" \
  --hand-debug-log "$RUN_DIR/hand_fidelity.jsonl" \
  "$@"

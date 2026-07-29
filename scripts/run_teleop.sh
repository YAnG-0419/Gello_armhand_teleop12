#!/usr/bin/env bash
# One-command teleop backend: fresh RUN_DIR and both debug logs.
# The operator GUI is a separate process (teleop_sources/gui/operator_gui.py).
#
# Exists because the equivalent multi-line paste has now buried three
# recordings: a clipboard missing its final newline leaves the command
# pending, and the next paste glues onto it. A single short command has no
# such failure mode.
#
# Extra arguments pass through to teleop_dual_fr3.py and later flags win,
# so e.g. `scripts/run_teleop.sh --hand-source pico` switches the hand
# source. TELEOP_RUN_DIR overrides the
# run directory.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_PARENT="/home/descfly/franka_teleop_data/diagnostics"
if [[ -n "${TELEOP_RUN_DIR:-}" ]]; then
  RUN_DIR="$TELEOP_RUN_DIR"
  if [[ -e "$RUN_DIR" ]]; then
    echo "Refusing to reuse TELEOP_RUN_DIR: $RUN_DIR" >&2
    exit 1
  fi
  if ! mkdir -- "$RUN_DIR"; then
    echo "Could not create TELEOP_RUN_DIR: $RUN_DIR" >&2
    exit 1
  fi
else
  mkdir -p -- "$RUN_PARENT"
  RUN_DIR="$(mktemp -d "$RUN_PARENT/$(date +%Y%m%d_%H%M%S).XXXXXX")"
fi
echo "RUN_DIR=$RUN_DIR"

cd "$REPO_ROOT"
# activate + exec, NOT `conda run`: conda run wraps python in a subprocess
# and dies first on Ctrl-C, orphaning python into the background where it
# cannot restore the terminal (tcsetattr EIO -> hidden cursor, stuck cbreak).
# With exec, python owns the foreground and cleanup completes properly.
CONDA_BASE="$(conda info --base)"
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate franka-teleop-pico
exec python teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py \
  --config config/pico.yaml --arm-source motion-trackers \
  --hand-source manus \
  --debug-log "$RUN_DIR/ee_jitter.jsonl" \
  --hand-debug-log "$RUN_DIR/hand_fidelity.jsonl" \
  "$@"

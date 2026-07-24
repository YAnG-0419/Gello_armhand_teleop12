#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec conda run --no-capture-output --name franka-teleop-pico \
  python "${repo}/teleop_sources/pico/scripts/hardware/teleop_dual_fr3.py" \
  "$@" --config "${repo}/config/pico.yaml"

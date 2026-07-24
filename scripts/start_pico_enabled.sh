#!/usr/bin/env bash
set -euo pipefail

if [[ ${FRANKA_TELEOP_ENABLE:-} != I_UNDERSTAND ]]; then
  echo "Set FRANKA_TELEOP_ENABLE=I_UNDERSTAND after completing dry-run validation." >&2
  exit 2
fi

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "${repo}/docker"
TELEOP_OUTPUT_ENABLED=true docker compose up teleop-control pico-bridge

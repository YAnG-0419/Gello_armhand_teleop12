#!/usr/bin/env bash
# Start the imported dual-FR3 MoveIt stack in an isolated operating mode.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
usage() {
  echo "Usage: $0 --fake|--real" >&2
}

if [[ $# -ne 1 || ( "$1" != "--fake" && "$1" != "--real" ) ]]; then
  usage
  exit 2
fi

cd "$REPO_ROOT/docker"
running="$(docker compose ps --services --status running)"
for service in franka-control fake-franka-control teleop-control gello-bridge pico-bridge vive-bridge moveit-fake moveit-real; do
  if grep -qx "$service" <<<"$running"; then
    echo "Refusing to start MoveIt while $service is running." >&2
    echo "Stop the Gello/teleop stack first; both modes own the FR3 command path." >&2
    exit 1
  fi
done

if [[ "$1" == "--real" ]]; then
  "$REPO_ROOT/ops/run/preflight.sh"
  service=moveit-real
else
  service=moveit-fake
fi

if [[ -n "${DISPLAY:-}" ]] && command -v xhost >/dev/null 2>&1; then
  xhost +local:root >/dev/null
fi
exec docker compose up "$service"

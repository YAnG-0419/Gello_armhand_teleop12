#!/usr/bin/env bash
# Gello arms plus optional MANUS-to-Wuji hands. O30i remains untouched/default.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICES=(franka-control teleop-control gello-bridge)
stack_started=false

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ "$stack_started" == true ]]; then
    (cd "$REPO_ROOT/docker" && docker compose stop --timeout 10 "${SERVICES[@]}") || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

running="$(cd "$REPO_ROOT/docker" && docker compose ps --services --status running)"
if grep -qx hand-control <<<"$running"; then
  echo "Refusing to start Wuji mode while the O30i/G20 hand-control service is running." >&2
  echo "Stop the existing teleop stack first. O30i configuration has not been changed." >&2
  exit 1
fi
if grep -Eq '^(moveit-fake|moveit-real)$' <<<"$running"; then
  echo "Refusing to start Wuji/Gello mode while a MoveIt service is running." >&2
  echo "Stop MoveIt first so only one controller stack owns the FR3 arms." >&2
  exit 1
fi

"$REPO_ROOT/scripts/preflight.sh"
cd "$REPO_ROOT/docker"
docker compose up -d "${SERVICES[@]}"
stack_started=true
docker compose ps "${SERVICES[@]}"

cd "$REPO_ROOT"
"$REPO_ROOT/scripts/run_operator.sh" \
  --hand-source wuji \
  "$@"

#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
"$REPO_ROOT/ops/run/preflight.sh"

cd "$REPO_ROOT/docker"
exec docker compose up \
  franka-control teleop-control gello-bridge hand-control

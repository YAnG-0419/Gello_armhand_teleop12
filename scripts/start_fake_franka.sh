#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "${repo}/docker"
FRANKA_ROBOT_CONFIG=/workspace/franka_upper_body_teleop/config/fake_workcell.yaml \
  docker compose up franka-control

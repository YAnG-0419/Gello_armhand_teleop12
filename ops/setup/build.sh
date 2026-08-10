#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
docker build --tag franka-upper-body-teleop:latest "${repo}/docker"
docker run --rm \
  --volume "${repo}:/workspace/franka_upper_body_teleop" \
  --workdir /workspace/franka_upper_body_teleop \
  franka-upper-body-teleop:latest \
  docker/build_workspace.sh "$@"

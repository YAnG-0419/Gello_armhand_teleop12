#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "${repo}/docker"
exec docker compose run --rm tools \
  ros2 service call /reset_to_initial_pose std_srvs/srv/Trigger '{}'

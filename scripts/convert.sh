#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "${repo}/docker"
exec docker compose run --rm tools ros2 run teleop_data convert "$@"

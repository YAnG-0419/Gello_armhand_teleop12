#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "${repo}/docker"
TELEOP_OUTPUT_ENABLED=false docker compose up teleop-control pico-bridge

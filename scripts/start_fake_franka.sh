#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
exec "${repo}/scripts/compose.sh" up fake-franka-control

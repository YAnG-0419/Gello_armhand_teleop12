#!/usr/bin/env bash
# Backward-compatible explicit name for the default VIVE + MANUS operator.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${repo_root}/scripts/run_teleop.sh" "$@"

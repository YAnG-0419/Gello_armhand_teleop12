#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

cd "${REPO_ROOT}"
exec conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/manus/scripts/diagnose_o30i_retarget.py "$@"

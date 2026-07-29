#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
O30_MANUS_DEBUG_LOG="${O30_MANUS_DEBUG_LOG:-/tmp/o30_manus_live.jsonl}"
O30_FILTER_ALPHA="${O30_FILTER_ALPHA:-0.85}"

cd "${REPO_ROOT}"
exec conda run --no-capture-output --name franka-teleop-pico \
  python teleop_sources/manus/scripts/teleop_o30i.py \
  --debug-log "${O30_MANUS_DEBUG_LOG}" \
  --filter-alpha "${O30_FILTER_ALPHA}" \
  "$@"

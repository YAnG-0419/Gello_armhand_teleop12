#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
viewer=/opt/OrbbecSDK_v2.9.3/tools/OrbbecViewer
viewer_dir=$(dirname "${viewer}")

if [[ ! -x ${viewer} ]]; then
  echo "Orbbec SDK v2 Viewer is missing: ${viewer}" >&2
  exit 1
fi
if docker compose --file "${repo}/docker/compose.yaml" \
    ps --status running --services | grep -Fxq orbbec; then
  echo "The ROS Orbbec service is running; stop it before opening Viewer:" >&2
  echo "  docker compose --file ${repo}/docker/compose.yaml stop orbbec" >&2
  exit 1
fi
if pgrep -x OrbbecViewer >/dev/null; then
  echo "OrbbecViewer is already running." >&2
  exit 1
fi

display=${DISPLAY:-:1}
xauthority=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
cd "${viewer_dir}"
exec env DISPLAY="${display}" XAUTHORITY="${xauthority}" "${viewer}"

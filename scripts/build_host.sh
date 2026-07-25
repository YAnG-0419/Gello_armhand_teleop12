#!/usr/bin/env bash
# Build the host-side hand stack: the vendored LinkerHand driver and the bridge.
#
# This is separate from scripts/build.sh, which builds ros_ws inside the Humble
# container. The hand stack must be built against the host's ROS distribution
# because the vendor driver needs direct SocketCAN access, and a container-built
# install tree cannot run on the host.
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [ -z "${ROS_DISTRO:-}" ]; then
  candidates=(/opt/ros/*/setup.bash)
  if [ ! -e "${candidates[0]}" ]; then
    echo "No ROS installation found under /opt/ros" >&2
    exit 1
  fi
  # ROS setup scripts reference unset variables, so relax -u around sourcing.
  set +u
  # shellcheck disable=SC1090
  source "${candidates[0]}"
  set -u
fi

echo "Building host workspace with ROS ${ROS_DISTRO}"
cd "${repo}/host_ws"

# --symlink-install is required: the vendored driver's setup.py does not install
# its LinkerHand/config/*.yaml into share/, so a copying install cannot find
# setting.yaml at runtime.
colcon build --symlink-install "$@"

cat <<EOF

Done. Source it with:

  source ${repo}/host_ws/install/setup.bash
EOF

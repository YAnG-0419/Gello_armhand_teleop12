#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
domain_id=${ROS_DOMAIN_ID:-231}
cd "${repo}/docker"

cleanup() {
  docker compose down --remove-orphans >/dev/null
}
trap cleanup EXIT

ROS_DOMAIN_ID="${domain_id}" TELEOP_OUTPUT_ENABLED=false \
  docker compose up -d teleop-control pico-bridge

docker compose run --rm -d tools bash -lc \
  'exec ros2 topic pub --rate 50 /left/franka/joint_states sensor_msgs/msg/JointState \
  "{name: [left_fr3_joint1, left_fr3_joint2, left_fr3_joint3, left_fr3_joint4, left_fr3_joint5, left_fr3_joint6, left_fr3_joint7], position: [-0.1064695120, 0.5432978868, -0.7150393724, -2.0424106121, 0.7801840901, 2.3256008625, 0.3307797611]}"'
docker compose run --rm -d tools bash -lc \
  'exec ros2 topic pub --rate 50 /right/franka/joint_states sensor_msgs/msg/JointState \
  "{name: [right_fr3_joint1, right_fr3_joint2, right_fr3_joint3, right_fr3_joint4, right_fr3_joint5, right_fr3_joint6, right_fr3_joint7], position: [-1.2620162964, -0.3890800774, 2.3749103546, -2.3138155937, -0.6073474288, 2.2486193180, 0.0537403040]}"'

sleep 2
(
  sleep 1
  PYTHONPATH="${repo}/ros_ws/src/teleop_core:${repo}/teleop_sources/pico/src" \
    python3 -c \
    "from pico_bimanual_franka_teleop.robot_udp import UdpRobotBackend
import time
b=UdpRobotBackend()
q=b.wait_for_state(3)
for _ in range(100):
    b.send_command(q, ('left','right'))
    time.sleep(0.01)
b.close()"
) &

validated=$(docker compose run --rm tools bash -lc \
  'timeout 8 ros2 topic echo --once /teleop/validated_arm_commands')
wait
grep -q left_fr3v2_joint1 <<<"${validated}"
grep -q right_fr3v2_joint7 <<<"${validated}"

set +e
docker compose run --rm tools bash -lc \
  'timeout 2 ros2 topic echo --once /target_robot/joint_commands' \
  >/dev/null 2>&1
status=$?
set -e
if [[ ${status} -ne 124 ]]; then
  echo "Dry-run unexpectedly published a hardware command." >&2
  exit 1
fi

echo "Dry-run passed: UDP, source adapter, and safety gateway work; hardware output stayed silent."

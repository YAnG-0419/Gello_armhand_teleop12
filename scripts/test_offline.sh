#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

PYTHONPATH="${repo}/ros_ws/src/teleop_core:${repo}/ros_ws/src/teleop_data" \
  pytest -q \
    "${repo}/ros_ws/src/teleop_core/test" \
    "${repo}/ros_ws/src/teleop_data/test/test_config.py" \
    "${repo}/ros_ws/src/teleop_data/test/test_timeseries.py"

conda run --no-capture-output --name franka-teleop-pico \
  pytest -q "${repo}/teleop_sources/pico/tests"
conda run --no-capture-output --name franka-teleop-pico \
  python "${repo}/teleop_sources/pico/scripts/simulation/teleop_dual_fr3_mujoco.py" \
    --headless --mock-xr --duration 2
conda run --no-capture-output --name franka-teleop-pico \
  python "${repo}/teleop_sources/pico/scripts/simulation/validate_dual_fr3_dynamics.py"

docker run --rm \
  --volume "${repo}:/workspace/franka_upper_body_teleop" \
  --workdir /workspace/franka_upper_body_teleop \
  franka-upper-body-teleop:latest \
  bash -lc \
  'source ros_ws/install/setup.bash &&
   cd ros_ws &&
   colcon test --packages-select teleop_core teleop_data pico_teleop_bridge franka_fr3_arm_controllers &&
   colcon test-result --verbose &&
   python3 -m pytest -q src/teleop_core/test src/teleop_data/test'

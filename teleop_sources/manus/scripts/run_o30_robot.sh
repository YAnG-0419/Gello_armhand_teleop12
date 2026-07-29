#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

O30_TICKS_AT_LOWER="${O30_TICKS_AT_LOWER:-0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}"
O30_TICKS_AT_UPPER="${O30_TICKS_AT_UPPER:-255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255}"
O30_MAX_COMMAND_RATE="${O30_MAX_COMMAND_RATE:-12.0}"
O30_COMMAND_TIMEOUT="${O30_COMMAND_TIMEOUT:-0.25}"
O30_STATE_TIMEOUT="${O30_STATE_TIMEOUT:-0.5}"

echo "Starting right O30i hardware control."
echo "Transport: libcanbus"
echo "Slew limit: ${O30_MAX_COMMAND_RATE} rad/s"
echo "Tick mapping: full vendor range 0..255 over each URDF joint range"
echo "The driver starts disabled and waits for fresh feedback and a valid command."

cd "${REPO_ROOT}/docker"
exec docker compose run --rm hand-control \
  ros2 launch linker_hand_bridge hands.launch.py \
  sides:=right \
  right_model:=o30i \
  o30_transport:=libcanbus \
  enabled:=true \
  o30_calibration_verified:=true \
  o30_tick_at_lower:="${O30_TICKS_AT_LOWER}" \
  o30_tick_at_upper:="${O30_TICKS_AT_UPPER}" \
  o30_command_timeout:="${O30_COMMAND_TIMEOUT}" \
  o30_state_timeout:="${O30_STATE_TIMEOUT}" \
  max_command_rate:="${O30_MAX_COMMAND_RATE}"

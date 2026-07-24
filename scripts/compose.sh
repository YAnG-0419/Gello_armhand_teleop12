#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
env_file="${repo}/docker/.env"
required_keys=(
  TELEOP_DATA_ROOT
  TELEOP_ROS_DOMAIN_ID
  FRANKA_CPUSET
  FRANKA_ROBOT_CONFIG
)

if [[ ! -f "${env_file}" ]]; then
  echo "Missing required configuration file: ${env_file}" >&2
  exit 2
fi

configured_keys=()
line_number=0
while IFS= read -r line || [[ -n "${line}" ]]; do
  line_number=$((line_number + 1))
  if [[ -z "${line}" || "${line}" == \#* ]]; then
    continue
  fi
  if [[ ! "${line}" =~ ^([A-Za-z_][A-Za-z0-9_]*)=(.+)$ ]]; then
    echo "Malformed or empty entry at ${env_file}:${line_number}" >&2
    exit 2
  fi
  configured_keys+=("${BASH_REMATCH[1]}")
done < "${env_file}"

for key in "${configured_keys[@]}"; do
  known=false
  for required in "${required_keys[@]}"; do
    if [[ "${key}" == "${required}" ]]; then
      known=true
      break
    fi
  done
  if [[ "${known}" != true ]]; then
    echo "Unknown configuration key in ${env_file}: ${key}" >&2
    exit 2
  fi
done

for key in "${required_keys[@]}"; do
  count=0
  for configured in "${configured_keys[@]}"; do
    if [[ "${configured}" == "${key}" ]]; then
      count=$((count + 1))
    fi
  done
  if [[ "${count}" -ne 1 ]]; then
    echo "${env_file} must contain exactly one non-empty ${key}=... entry" >&2
    exit 2
  fi
  unset "${key}"
done

cd "${repo}/docker"
exec docker compose --env-file "${env_file}" "$@"

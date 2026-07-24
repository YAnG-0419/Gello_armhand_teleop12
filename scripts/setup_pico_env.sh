#!/usr/bin/env bash
set -euo pipefail

repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
environment=franka-teleop-pico

if conda env list | awk '{print $1}' | grep -Fxq "${environment}"; then
  conda env update --name "${environment}" \
    --file "${repo}/teleop_sources/pico/environment.yml" --prune
else
  conda env create --file "${repo}/teleop_sources/pico/environment.yml"
fi

conda run --name "${environment}" python -m pip install \
  "${repo}/third_party/xrobotoolkit_sdk"

conda run --name "${environment}" python -m pip install \
  -e "${repo}/ros_ws/src/teleop_core" \
  -e "${repo}/teleop_sources/pico"

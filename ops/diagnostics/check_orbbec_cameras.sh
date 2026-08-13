#!/usr/bin/env bash
# Read-only check of the Ethernet Gemini 435Le and USB Gemini 305.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
ENV_FILE="$REPO_ROOT/docker/.env"
CAMERA_IP="${ORBBEC_NET_DEVICE_IP:-}"
CAMERA_PORT="${ORBBEC_NET_DEVICE_PORT:-8090}"
if [[ -z "$CAMERA_IP" && -f "$ENV_FILE" ]]; then
  CAMERA_IP="$(awk -F= '$1 == "ORBBEC_NET_DEVICE_IP" {print $2; exit}' "$ENV_FILE")"
  CAMERA_PORT="$(awk -F= '$1 == "ORBBEC_NET_DEVICE_PORT" {print $2; exit}' "$ENV_FILE")"
fi
CAMERA_IP="${CAMERA_IP:-192.168.35.35}"
CAMERA_PORT="${CAMERA_PORT:-8090}"

fail=0
warn() { echo "[WARN] $*"; }
pass() { echo "[PASS] $*"; }
bad() { echo "[FAIL] $*"; fail=1; }

if ping -c 1 -W 1 "$CAMERA_IP" >/dev/null 2>&1 \
    && nc -z -w 2 "$CAMERA_IP" "$CAMERA_PORT" >/dev/null 2>&1; then
  pass "Gemini 435Le reachable at ${CAMERA_IP}:${CAMERA_PORT}"
else
  bad "Gemini 435Le not reachable at ${CAMERA_IP}:${CAMERA_PORT}"
fi

if lsusb | grep -qiE '2bc5|orbbec'; then
  pass "Gemini 305 USB device present"
  lsusb | grep -iE '2bc5|orbbec' || true
else
  warn "Gemini 305 USB not detected (VID 2bc5). Plug the Type-C cable into a USB 3.0 port."
fi

if command -v docker >/dev/null && docker info >/dev/null 2>&1; then
  (
    cd "$REPO_ROOT/docker"
    running="$(docker compose ps --services --status running 2>/dev/null || true)"
    if grep -qx orbbec <<<"$running"; then
      pass "orbbec service is running"
    else
      warn "orbbec service is not running"
    fi
    if grep -qx orbbec-305 <<<"$running"; then
      pass "orbbec-305 service is running"
    else
      warn "orbbec-305 service is not running"
    fi
  )
fi

exit "$fail"

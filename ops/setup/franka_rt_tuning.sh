#!/usr/bin/env bash
set -euo pipefail

STATE_DIR=/run/franka-rt-tuning
ETHTOOL=/usr/sbin/ethtool
LEFT_DEVICE=enp2s0f0
RIGHT_DEVICE=enp2s0f1
LEFT_IRQ_CPU=11
RIGHT_IRQ_CPU=15
RT_CPUS=(8 9 10 11 12 13 14 15)
# C-states deeper than this exit latency (us) are disabled on RT CPUs: C6/C8/C10
# wakeups (220-680 us) eat most of the 1 ms FCI cycle budget and the isolated
# IRQ cores drop into C10 between every 1 kHz packet.
MAX_CSTATE_LATENCY_US=10

require_root() {
  if [[ $EUID -ne 0 ]]; then
    echo "franka_rt_tuning.sh must run as root" >&2
    exit 1
  fi
}

device_irqs() {
  local device=$1
  awk -F: -v device="$device" \
    '$0 ~ device {gsub(/[[:space:]]/, "", $1); print $1}' /proc/interrupts
}

wait_for_device_irqs() {
  local device=$1
  local attempt
  for ((attempt = 0; attempt < 300; attempt++)); do
    if [[ -e "/sys/class/net/$device" ]] && device_irqs "$device" | read -r; then
      return 0
    fi
    sleep 0.1
  done
  echo "No IRQs found for $device after 30 seconds" >&2
  return 1
}

cstate_disable_paths() {
  local cpu=$1
  local state latency
  for state in "/sys/devices/system/cpu/cpu$cpu/cpuidle"/state*; do
    [[ -r "$state/latency" ]] || continue
    latency=$(<"$state/latency")
    if ((latency > MAX_CSTATE_LATENCY_US)); then
      printf '%s\n' "$state/disable"
    fi
  done
}

restore_state() {
  local irq old cpu path device eee rx_usecs gro gso tso
  if [[ -f "$STATE_DIR/irq-affinity.before" ]]; then
    while read -r irq old; do
      [[ -n "$irq" && -n "$old" ]] || continue
      path="/proc/irq/$irq/smp_affinity_list"
      [[ -w "$path" ]] && printf '%s\n' "$old" > "$path"
    done < "$STATE_DIR/irq-affinity.before"
  fi
  if [[ -f "$STATE_DIR/min-frequency.before" ]]; then
    while read -r cpu old; do
      [[ -n "$cpu" && -n "$old" ]] || continue
      path="/sys/devices/system/cpu/cpu$cpu/cpufreq/scaling_min_freq"
      [[ -w "$path" ]] && printf '%s\n' "$old" > "$path"
    done < "$STATE_DIR/min-frequency.before"
  fi
  if [[ -f "$STATE_DIR/nic-latency.before" ]]; then
    while read -r device eee rx_usecs gro gso tso; do
      [[ -n "$device" ]] || continue
      "$ETHTOOL" --set-eee "$device" eee "$eee"
      "$ETHTOOL" -C "$device" rx-usecs "$rx_usecs"
      "$ETHTOOL" -K "$device" gro "$gro" gso "$gso" tso "$tso"
    done < "$STATE_DIR/nic-latency.before"
  fi
  if [[ -f "$STATE_DIR/cpuidle-disable.before" ]]; then
    while read -r path old; do
      [[ -n "$path" && -n "$old" ]] || continue
      [[ -w "$path" ]] && printf '%s\n' "$old" > "$path"
    done < "$STATE_DIR/cpuidle-disable.before"
  fi
}

start_tuning() {
  local device target irq path old cpu min_path max_path maximum
  local eee rx_usecs gro gso tso
  rm -rf "$STATE_DIR"
  install -d -m 0755 "$STATE_DIR"
  : > "$STATE_DIR/irq-affinity.before"
  : > "$STATE_DIR/min-frequency.before"
  : > "$STATE_DIR/nic-latency.before"
  : > "$STATE_DIR/cpuidle-disable.before"
  trap 'restore_state' ERR

  for device in "$LEFT_DEVICE" "$RIGHT_DEVICE"; do
    wait_for_device_irqs "$device"
    if [[ "$device" == "$LEFT_DEVICE" ]]; then
      target=$LEFT_IRQ_CPU
    else
      target=$RIGHT_IRQ_CPU
    fi
    mapfile -t irqs < <(device_irqs "$device")
    for irq in "${irqs[@]}"; do
      path="/proc/irq/$irq/smp_affinity_list"
      old=$(<"$path")
      printf '%s %s\n' "$irq" "$old" >> "$STATE_DIR/irq-affinity.before"
      printf '%s\n' "$target" > "$path"
    done

    eee=$("$ETHTOOL" --show-eee "$device" | awk '/EEE status:/ {print $3; exit}')
    [[ "$eee" == "enabled" ]] && eee=on || eee=off
    rx_usecs=$("$ETHTOOL" -c "$device" | awk '$1 == "rx-usecs:" {print $2; exit}')
    gro=$("$ETHTOOL" -k "$device" | awk '$1 == "generic-receive-offload:" {print $2; exit}')
    gso=$("$ETHTOOL" -k "$device" | awk '$1 == "generic-segmentation-offload:" {print $2; exit}')
    tso=$("$ETHTOOL" -k "$device" | awk '$1 == "tcp-segmentation-offload:" {print $2; exit}')
    printf '%s %s %s %s %s %s\n' \
      "$device" "$eee" "$rx_usecs" "$gro" "$gso" "$tso" \
      >> "$STATE_DIR/nic-latency.before"
    "$ETHTOOL" --set-eee "$device" eee off
    "$ETHTOOL" -C "$device" rx-usecs 0
    "$ETHTOOL" -K "$device" gro off gso off tso off
  done

  for cpu in "${RT_CPUS[@]}"; do
    min_path="/sys/devices/system/cpu/cpu$cpu/cpufreq/scaling_min_freq"
    max_path="/sys/devices/system/cpu/cpu$cpu/cpufreq/scaling_max_freq"
    if [[ ! -w "$min_path" || ! -r "$max_path" ]]; then
      echo "CPU frequency controls unavailable for CPU $cpu" >&2
      return 1
    fi
    old=$(<"$min_path")
    maximum=$(<"$max_path")
    printf '%s %s\n' "$cpu" "$old" >> "$STATE_DIR/min-frequency.before"
    printf '%s\n' "$maximum" > "$min_path"
  done

  for cpu in "${RT_CPUS[@]}"; do
    while read -r path; do
      if [[ ! -w "$path" ]]; then
        echo "cpuidle control unavailable: $path" >&2
        return 1
      fi
      old=$(<"$path")
      printf '%s %s\n' "$path" "$old" >> "$STATE_DIR/cpuidle-disable.before"
      printf '1\n' > "$path"
    done < <(cstate_disable_paths "$cpu")
  done

  date --iso-8601=seconds > "$STATE_DIR/started-at"
  trap - ERR
}

stop_tuning() {
  restore_state
  rm -rf "$STATE_DIR"
}

show_status() {
  local device irq cpu path
  for device in "$LEFT_DEVICE" "$RIGHT_DEVICE"; do
    while read -r irq; do
      printf '%s irq=%s affinity=' "$device" "$irq"
      awk '{print}' "/proc/irq/$irq/smp_affinity_list"
    done < <(device_irqs "$device")
    "$ETHTOOL" --show-eee "$device" | awk '/EEE status:/'
    "$ETHTOOL" -c "$device" | awk '$1 == "rx-usecs:"'
    "$ETHTOOL" -k "$device" | awk \
      '/^(generic-receive-offload|generic-segmentation-offload|tcp-segmentation-offload):/'
  done
  for cpu in "${RT_CPUS[@]}"; do
    printf 'cpu%s min=' "$cpu"
    awk '{print}' "/sys/devices/system/cpu/cpu$cpu/cpufreq/scaling_min_freq"
  done
  for cpu in "${RT_CPUS[@]}"; do
    while read -r path; do
      printf 'cpu%s %s disable=' "$cpu" "$(basename "$(dirname "$path")")"
      awk '{print}' "$path"
    done < <(cstate_disable_paths "$cpu")
  done
}

require_root
case "${1:-}" in
  start) start_tuning ;;
  stop) stop_tuning ;;
  status) show_status ;;
  *)
    echo "usage: $0 {start|stop|status}" >&2
    exit 2
    ;;
esac

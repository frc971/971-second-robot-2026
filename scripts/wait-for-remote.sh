#!/usr/bin/env bash
# Usage: ./scripts/wait-for-remote.sh [host-or-ip] [interval-seconds]

set -eu

target="${1:-10.9.71.2}"
interval="${2:-1}"

echo "Waiting for remote $target at interval $interval seconds..."

while true; do
  # send a single ping; use default system behaviour
  if ping -c 1 "$target" >/dev/null 2>&1; then
    # success: exit 0 so && chains
    echo "Remote found!"
    break
  fi
  sleep "$interval"
done

# Warn when data logging on the roboRIO will not persist across power cycles.
issh_cmd="ssh -o ConnectTimeout=5 admin@$target"

# 1. Almost out of disk space
disk_usage=$($issh_cmd "df /home/lvuser | awk 'NR==2 {print \$5+0}'" 2>/dev/null || true)
if [ -n "$disk_usage" ] && [ "$disk_usage" -ge 90 ]; then
  echo "WARNING: roboRIO disk ${disk_usage}% full; logs will not be saved."
fi

# 2. Incorrect system timestamp
remote_now=$($issh_cmd "date +%s" 2>/dev/null || true)
if [ -n "$remote_now" ]; then
  skew=$(( remote_now - $(date +%s) ))
  if [ "${skew#-}" -gt 60 ]; then
    echo "WARNING: roboRIO time off by ${skew#-}s ($remote_now epoch vs host $(date +%s)); logs will not be saved."
  fi
fi

exit 0

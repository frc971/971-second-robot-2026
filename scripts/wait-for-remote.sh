#!/usr/bin/env bash
# Usage: ./scripts/wait-for-remote.sh [host-or-ip] [interval-seconds]

set -eu

target="${1:-10.9.71.2}"
interval="${2:-1}"

MIN_FREE_MB=500

echo "Waiting for remote $target at interval $interval seconds..."

while true; do
  if ping -c 1 "$target" >/dev/null 2>&1; then
    echo "Remote found!"

    free_mb=$(ssh "$target" "df -Pm / | awk 'NR==2 {print \$4}'")

    if [ "$free_mb" -lt "$MIN_FREE_MB" ]; then
      echo "ERROR: RoboRIO only has ${free_mb} MB free."
      echo "Refusing to start because log files may not be saved."
      exit 1
    fi

    echo "Disk space OK: ${free_mb} MB free."

    remote_epoch=$(ssh "$target" "date +%s")
    local_epoch=$(date +%s)

    max_skew=60
    skew=$((remote_epoch - local_epoch))
    skew=${skew#-}

    if [ "$skew" -gt "$max_skew" ]; then
      echo "ERROR: RoboRIO system timestamp is incorrect."
      echo "Clock difference: ${skew}s"
      echo "Open Driver Station on a Windows computer with the correct time"
      echo "to synchronize the RoboRIO clock."
      exit 1
    fi

    echo "System timestamp OK."

    echo "Remote is ready!"
    exit 0
  fi

  sleep "$interval"
done
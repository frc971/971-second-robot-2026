#!/usr/bin/env bash
# Usage: ./scripts/wait-for-remote.sh [host-or-ip] [interval-seconds]

set -eu

target="${1:-10.99.72.2}"
interval="${2:-1}"

echo "Waiting for remote $target at interval $interval seconds..."

while true; do
  # send a single ping; use default system behaviour
  if ping -c 1 "$target" >/dev/null 2>&1; then
    # success: exit 0 so && chains
    echo "Remote found!"
    exit 0
  fi
  sleep "$interval"
done

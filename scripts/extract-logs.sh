#!/usr/bin/env bash
# Usage: ./scripts/extract-logs.sh [count] [destination-directory]

REMOTE="admin@10.9.71.2"
LOG_DIR="/home/lvuser/logs"
COUNT="${1:-1}"                 # default = 1 log
DEST_DIR="${2:-$HOME/logs}"

mkdir -p "$DEST_DIR"

# Get the last N logs by modification time (newest first)
paths=$(ssh "$REMOTE" "ls -t $LOG_DIR/*.wpilog 2>/dev/null | head -n $COUNT")

# Always ensure at least the latest log if nothing found
if [ -z "$paths" ]; then
  echo "No logs found."
  exit 1
fi

count=$(echo "$paths" | wc -l)
echo "Downloading $count log(s) to $DEST_DIR..."

# Copy each file with scp
echo "$paths" | while read -r path; do
  scp "$REMOTE:$path" "$DEST_DIR/"
done
#!/usr/bin/env bash
# Usage: ./scripts/extract-logs.sh [minutes] [destination-directory]

REMOTE="admin@10.99.72.2"
LOG_DIR="/home/lvuser/logs"
MINUTES="${1:-0}"
DEST_DIR="${2:-$HOME/logs}"
mkdir -p "$DEST_DIR"

# Get logs from last N minutes (or just latest if 0)
if [ "$MINUTES" -eq 0 ]; then
  paths=$(ssh "$REMOTE" "ls -t $LOG_DIR/*.wpilog | head -n1")
else
  paths=$(ssh "$REMOTE" "find $LOG_DIR -name '*.wpilog' -mmin -$MINUTES")
fi

# Always ensure at least the latest log if nothing found
if [ -z "$paths" ]; then
  paths=$(ssh "$REMOTE" "ls -t $LOG_DIR/*.wpilog | head -n1")
fi

count=$(echo "$paths" | wc -l)
echo "Downloading $count log(s) to $DEST_DIR..."

# Copy each file with scp
echo "$paths" | while read -r path; do
  scp "$REMOTE:$path" "$DEST_DIR/"
done

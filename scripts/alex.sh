#!/usr/bin/env bash
# Usage: ./robot-deploy.sh [log_count]

set -euo pipefail

LOG_COUNT="${1:-3}"

./scripts/wait-for-remote.sh

echo "==> Extracting ${LOG_COUNT} log(s)..."
./scripts/extract-logs.sh "$LOG_COUNT"

./gradlew deploy

echo "==> Done."
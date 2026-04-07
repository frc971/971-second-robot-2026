#!/usr/bin/env bash
# Usage: ./robot-deploy.sh [log_count]

set -euo pipefail

LOG_COUNT="${1:-3}"

echo "==> Waiting for robot to come online..."
./scripts/wait-for-remote.sh

echo "==> Deploying via Gradle..."
./gradlew deploy

echo "==> Extracting ${LOG_COUNT} log(s)..."
./scripts/extract-logs.sh "$LOG_COUNT"

echo "==> Done."
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Jetson Nano deploy ==="

# Build
echo "[1/2] Building Docker image..."
docker compose -f docker-compose.jetson.yml build --no-cache

# Run
echo "[2/2] Starting stack..."
docker compose -f docker-compose.jetson.yml up -d

echo ""
echo "Dashboard:  http://$(hostname -I | awk '{print $1}'):8080"
echo "Logs:       docker logs -f car-calib-jetson"
echo "Stop:       docker compose -f docker-compose.jetson.yml down"

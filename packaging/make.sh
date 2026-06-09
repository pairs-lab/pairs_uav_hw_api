#!/usr/bin/env bash
# Host entrypoint: builds ros-jazzy-pairs-uav-hw-api_*.deb (+ PAIRS deps) via Docker.
# Depends on pairs_lib + pairs_msgs, both checked out next to this repo on `ros2`.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
OUTPUT_DIR="$SCRIPT_DIR/output"; IMAGE="pairs-hwapi-jazzy-packaging"
mkdir -p "$OUTPUT_DIR"
docker build -t "$IMAGE" "$SCRIPT_DIR"
docker run --rm -v "$SRC_DIR":/src:ro -v "$OUTPUT_DIR":/output "$IMAGE"
ls -1 "$OUTPUT_DIR"/*.deb

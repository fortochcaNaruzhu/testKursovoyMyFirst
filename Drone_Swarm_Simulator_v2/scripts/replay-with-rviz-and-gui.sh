#!/usr/bin/env bash
#
# One-command launch: replay container (with --interactive) + second container
# with RViz and the replay GUI. No ROS required on host.
#
# Usage: ./scripts/replay-with-rviz-and-gui.sh EXPERIMENT_PATH [RATE]
#   EXPERIMENT_PATH  Path to experiment dir (e.g. experiments/2026-02-28_18-35-11)
#   RATE             Optional playback speed, 0.25–4.0 (default 1.0)
#
# Requires: Docker, X11 (xhost +local:docker, DISPLAY=:0)
# Run from project root.
# Uses custom image swarm-replay-rviz-gui:noetic (ROS + python3-tk); builds once if missing.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}"
REPLAY_CONTAINER_NAME="swarm-replay-oneshot"
RVIZ_GUI_IMAGE="swarm-replay-rviz-gui:noetic"

if [ -z "${1:-}" ]; then
  echo "Usage: $0 EXPERIMENT_PATH [RATE]" >&2
  echo "  EXPERIMENT_PATH  path to experiment dir (e.g. experiments/2026-02-28_18-35-11)" >&2
  echo "  RATE             optional speed 0.25–4.0 (default 1.0)" >&2
  exit 1
fi

EXP_PATH="$1"
RATE="${2:-1.0}"

if ! [[ "$EXP_PATH" =~ ^[a-zA-Z0-9/_.-]+$ ]]; then
  echo "Error: EXPERIMENT_PATH may only contain letters, digits, /, _, ., -" >&2
  exit 1
fi

if ! [[ "$RATE" =~ ^[0-9]+\.?[0-9]*$ ]]; then
  echo "Error: RATE must be a positive number (e.g. 1.0 or 2)" >&2
  exit 1
fi
if ! awk "BEGIN { exit !($RATE > 0) }" 2>/dev/null; then
  echo "Error: RATE must be greater than 0" >&2
  exit 1
fi

if [[ "$EXP_PATH" == /* ]] && [[ "$EXP_PATH" == "$WORKSPACE"/* ]]; then
  EXP_PATH="${EXP_PATH#$WORKSPACE/}"
fi

export DISPLAY="${DISPLAY:-:0}"
xhost +local:docker 2>/dev/null || true

docker rm -f "$REPLAY_CONTAINER_NAME" 2>/dev/null || true

echo "Starting replay container (background, interactive)..."
docker run -d --name "$REPLAY_CONTAINER_NAME" \
  -e DISPLAY="$DISPLAY" \
  -e EXP="$EXP_PATH" \
  -e RATE="$RATE" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${WORKSPACE}:/workspace" \
  --network host \
  osrf/ros:noetic-desktop-full \
  bash -c '
    source /opt/ros/noetic/setup.bash
    roscore &
    sleep 5
    cd /workspace && python3 replay/replay_rviz.py --experiment "$EXP" --rate "$RATE" --interactive
  '

sleep 6
if ! docker image inspect "$RVIZ_GUI_IMAGE" >/dev/null 2>&1; then
  echo "Building image $RVIZ_GUI_IMAGE (one-time, has python3-tk)..."
  docker build -f "${PROJECT_ROOT}/docker/Dockerfile.replay-rviz-gui" -t "$RVIZ_GUI_IMAGE" "${PROJECT_ROOT}"
fi
echo "Starting RViz + GUI container (close RViz to stop)..."
docker run -it --rm \
  -e DISPLAY="$DISPLAY" \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e EXP="$EXP_PATH" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${WORKSPACE}:/workspace" \
  --network host \
  "$RVIZ_GUI_IMAGE" \
  bash -c '
    source /opt/ros/noetic/setup.bash
    python3 /workspace/replay/replay_gui.py --experiment "/workspace/$EXP" &
    sleep 2
    rosrun rviz rviz
  '

echo "Stopping replay container..."
docker stop "$REPLAY_CONTAINER_NAME" 2>/dev/null || true
docker rm "$REPLAY_CONTAINER_NAME" 2>/dev/null || true

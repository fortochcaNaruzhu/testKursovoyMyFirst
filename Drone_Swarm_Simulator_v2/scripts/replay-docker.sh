#!/usr/bin/env bash
#
# Replay drone swarm experiment in RViz via Docker (ROS Noetic).
# Launches roscore and replay_rviz in a single container; mounts project as /workspace.
#
# Usage: ./scripts/replay-docker.sh EXPERIMENT_PATH [RATE] [INTERACTIVE]
#   EXPERIMENT_PATH  Path to experiment dir (e.g. experiments/2026-02-28_18-35-11)
#   RATE             Optional playback speed, 0.25–4.0 (default 1.0)
#   INTERACTIVE      Optional: 1, true, or yes to enable --interactive (play/pause/seek/speed via ROS topics; no keyboard in Docker)
#
# Requires: Docker, X11 (xhost +local:docker, DISPLAY=:0)
# Run from project root so replay/ and experiments/ exist.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}"

if [ -z "${1:-}" ]; then
  echo "Usage: $0 EXPERIMENT_PATH [RATE] [INTERACTIVE]" >&2
  echo "  EXPERIMENT_PATH  path to experiment dir (e.g. experiments/2026-02-28_18-35-11)" >&2
  echo "  RATE             optional speed 0.25–4.0 (default 1.0)" >&2
  echo "  INTERACTIVE      1, true, or yes to enable play/pause/seek/speed via ROS topics" >&2
  exit 1
fi

EXP_PATH="$1"
RATE="${2:-1.0}"
INTERACTIVE_ARG=""
case "${3:-}" in
  1|true|yes) INTERACTIVE_ARG="--interactive" ;;
esac

# Sanitize: no shell metacharacters (safe for -e and python args)
if ! [[ "$EXP_PATH" =~ ^[a-zA-Z0-9/_.-]+$ ]]; then
  echo "Error: EXPERIMENT_PATH may only contain letters, digits, /, _, ., -" >&2
  exit 1
fi

# RATE must be a positive number (no eval of user input; regex restricts to digits/dot)
if ! [[ "$RATE" =~ ^[0-9]+\.?[0-9]*$ ]]; then
  echo "Error: RATE must be a positive number (e.g. 1.0 or 2)" >&2
  exit 1
fi
if ! awk "BEGIN { exit !($RATE > 0) }" 2>/dev/null; then
  echo "Error: RATE must be greater than 0" >&2
  exit 1
fi

# If absolute path, make it relative to workspace for use inside container
if [[ "$EXP_PATH" == /* ]]; then
  if [[ "$EXP_PATH" == "$WORKSPACE"/* ]]; then
    EXP_PATH="${EXP_PATH#$WORKSPACE/}"
  fi
fi

export DISPLAY="${DISPLAY:-:0}"
xhost +local:docker 2>/dev/null || true

docker run -it --rm \
  -e DISPLAY="$DISPLAY" \
  -e EXP="$EXP_PATH" \
  -e RATE="$RATE" \
  -e INTERACTIVE_ARG="$INTERACTIVE_ARG" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${WORKSPACE}:/workspace" \
  --network host \
  osrf/ros:noetic-desktop-full \
  bash -c '
    source /opt/ros/noetic/setup.bash
    roscore &
    sleep 5
    cd /workspace && python3 replay/replay_rviz.py --experiment "$EXP" --rate "$RATE" $INTERACTIVE_ARG
  '

#!/usr/bin/env bash
#
# Host-only: start replay with --interactive in background, then run GUI.
# Replay exits when the GUI window is closed.
#
# Prerequisite: roscore must be running (e.g. in another terminal).
# Usage: ./scripts/replay-with-gui.sh EXPERIMENT_PATH [RATE]
#
# Does not modify replay-with-rviz.sh or replay-docker.sh.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$PROJECT_ROOT"

if [ -z "${1:-}" ]; then
  echo "Usage: $0 EXPERIMENT_PATH [RATE]" >&2
  echo "  EXPERIMENT_PATH  path to experiment dir (e.g. experiments/2026-02-28_18-35-11)" >&2
  echo "  RATE            optional speed 0.25–4.0 (default 1.0)" >&2
  echo "Prerequisite: run roscore in another terminal first." >&2
  exit 1
fi

EXP_PATH="$1"
RATE="${2:-1.0}"

if [ -n "${ROS_DISTRO:-}" ]; then
  : # already sourced
else
  if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
  else
    echo "Source ROS first, e.g. source /opt/ros/noetic/setup.bash" >&2
    exit 1
  fi
fi

python3 replay/replay_rviz.py --experiment "$EXP_PATH" --rate "$RATE" --interactive &
REPLAY_PID=$!
trap 'kill $REPLAY_PID 2>/dev/null' EXIT
sleep 2
python3 replay/replay_gui.py --experiment "$EXP_PATH"

#!/usr/bin/env python3
"""
ROS node for 3D replay of drone swarm experiments in RViz.

Loads experiment data via replay.csv_loader, publishes PoseStamped for each drone
on /swarm/drone_<id>/pose at a configurable playback rate. Supports interactive
mode (play/pause, seek, speed) via keyboard and ROS topics. Uses only replay.csv_loader
and rospy; no imports from core/ or scenarios/.
"""

import argparse
import json
import logging
import os
import select
import signal
import sys
import threading
import tty
import termios
from typing import Any, Dict, List, Optional, Tuple

# Ensure project root is on path when run as script (replay/replay_rviz.py or from replay/)
_script_dir = os.path.dirname(os.path.abspath(__file__))
_replay_parent = os.path.dirname(_script_dir)
if _replay_parent not in sys.path:
    sys.path.insert(0, _replay_parent)

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Empty, Float32, Float64, String
    from tf.transformations import quaternion_from_euler
except ImportError as e:
    print(
        "replay_rviz.py requires ROS and rospy. Install ROS (e.g. Noetic) and source setup.bash:\n"
        "  source /opt/ros/noetic/setup.bash\n"
        "Error: " + str(e),
        file=sys.stderr,
    )
    sys.exit(1)

from replay.csv_loader import load_experiment, iter_steps
from replay.playback_controller import PlaybackState, SPEED_MIN, SPEED_MAX

logger = logging.getLogger(__name__)


def _ned_to_enu(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert NED coordinates to ENU for RViz (x→y, y→x, z→-z).

    Args:
        x, y, z: Position in NED frame (meters).

    Returns:
        (enu_x, enu_y, enu_z) for RViz display.
    """
    return (float(y), float(x), float(-z))


def _row_to_pose_stamped(
    row: Dict[str, Any],
    frame_id: str = "world",
    stamp: Optional[Any] = None,
) -> PoseStamped:
    """Build PoseStamped from CSV row (t,x,y,z,rx,ry,rz,hasCollision). NED → ENU.

    Args:
        row: CSV row dict with x, y, z, rx, ry, rz.
        frame_id: ROS frame ID for the pose header.
        stamp: Optional ROS time stamp; uses rospy.Time.now() if None.

    Returns:
        PoseStamped message with position (ENU) and quaternion orientation.
    """
    enu_x, enu_y, enu_z = _ned_to_enu(
        float(row["x"]), float(row["y"]), float(row["z"])
    )
    roll = float(row["rx"])
    pitch = float(row["ry"])
    yaw = float(row["rz"])
    q = quaternion_from_euler(roll, pitch, yaw)
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp or rospy.Time.now()
    msg.pose.position.x = enu_x
    msg.pose.position.y = enu_y
    msg.pose.position.z = enu_z
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]
    return msg


def _keyboard_loop(state: PlaybackState, step_times: List[float]) -> None:
    """Background thread: read keys and update state. Uses raw stdin.

    Args:
        state: Shared playback state to update (play/pause, index, speed).
        step_times: List of step times for seek (unused; reserved for future use).
    """
    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not rospy.is_shutdown():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            key = sys.stdin.read(1)
            if key == " ":
                state.toggle_playing()
            elif key == "q" or key == "\x03":
                rospy.signal_shutdown("keyboard quit")
                break
            elif key in ("+", "="):
                state.adjust_speed(0.25)
            elif key == "-":
                state.adjust_speed(-0.25)
            elif key == "\x1b":
                # Escape sequence (arrows)
                more = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.05)[0] else ""
                seq = key + more
                if seq == "\x1b[D":
                    state.set_index(state.get_state()[1] - 1)
                elif seq == "\x1b[C":
                    state.set_index(state.get_state()[1] + 1)
            elif key == "a":
                state.set_index(state.get_state()[1] - 1)
            elif key == "d":
                state.set_index(state.get_state()[1] + 1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _parse_args() -> argparse.Namespace:
    """Parse command-line arguments for replay node.

    Returns:
        Parsed namespace with experiment, rate, interactive.
    """
    parser = argparse.ArgumentParser(
        description="Replay drone swarm experiment in RViz via ROS topics."
    )
    parser.add_argument(
        "--experiment",
        type=str,
        required=True,
        help="Path to experiment directory (contains metadata.json and drone_*.csv).",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Playback speed multiplier (1.0 = real-time). With --interactive, initial speed.",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Enable play/pause, seek, speed via keyboard and ROS topics.",
    )
    return parser.parse_args()


def _run_linear(
    steps: List[Tuple[float, List[Optional[Dict[str, Any]]]]],
    pose_pubs: Dict[int, Any],
    num_drones: int,
    rate: float,
) -> None:
    """Run single linear playback (no interactive controls).

    Args:
        steps: List of (time, list of row dicts per drone).
        pose_pubs: Map drone_id -> Publisher for PoseStamped.
        num_drones: Number of drones.
        rate: Playback speed multiplier.
    """
    prev_t: float = 0.0
    for t, step_list in steps:
        if rospy.is_shutdown():
            break
        stamp = rospy.Time.now()
        for i, row in enumerate(step_list):
            if row is None:
                continue
            drone_id = i + 1
            msg = _row_to_pose_stamped(row, stamp=stamp)
            pose_pubs[drone_id].publish(msg)
        dt = t - prev_t
        prev_t = t
        if dt > 0 and rate > 0:
            rospy.sleep(dt / rate)
    rospy.loginfo("Replay finished.")


def _run_interactive(
    steps: List[Tuple[float, List[Optional[Dict[str, Any]]]]],
    pose_pubs: Dict[int, Any],
    meta_pub: Any,
    metadata: Dict[str, Any],
    num_drones: int,
    state: PlaybackState,
    use_keyboard: bool,
) -> None:
    """Seekable loop: read state, publish current step, advance or re-publish when paused.

    Args:
        steps: List of (time, list of row dicts per drone).
        pose_pubs: Map drone_id -> Publisher for PoseStamped.
        meta_pub: Publisher for metadata JSON string.
        metadata: Experiment metadata dict.
        num_drones: Number of drones.
        state: Thread-safe playback state (play/pause, index, speed).
        use_keyboard: Whether to start keyboard control thread.
    """
    step_times = [s[0] for s in steps]
    n = len(steps)

    # ROS subscribers for controls
    def on_play(_: Any) -> None:
        state.set_playing(True)

    def on_pause(_: Any) -> None:
        state.set_playing(False)

    def on_seek(msg: Float64) -> None:
        state.seek_to_time(float(msg.data), step_times)

    def on_speed(msg: Float32) -> None:
        state.set_speed(float(msg.data))

    rospy.Subscriber("/replay/play", Empty, on_play)
    rospy.Subscriber("/replay/pause", Empty, on_pause)
    rospy.Subscriber("/replay/seek", Float64, on_seek)
    rospy.Subscriber("/replay/speed", Float32, on_speed)

    if use_keyboard:
        kb_thread = threading.Thread(
            target=_keyboard_loop, args=(state, step_times), daemon=True
        )
        kb_thread.start()
        rospy.loginfo(
            "Keyboard: Space=play/pause, a/d or Left/Right=step, +/-=speed, q=quit"
        )

    # Publish metadata once
    meta_pub.publish(String(data=json.dumps(metadata)))

    while not rospy.is_shutdown():
        playing, idx, speed = state.get_state()
        t, step_list = steps[idx]
        stamp = rospy.Time.now()
        for i, row in enumerate(step_list):
            if row is None:
                continue
            drone_id = i + 1
            msg = _row_to_pose_stamped(row, stamp=stamp)
            pose_pubs[drone_id].publish(msg)

        if playing and idx < n - 1:
            next_t = steps[idx + 1][0]
            dt = next_t - t
            if dt > 0 and speed > 0:
                rospy.sleep(dt / speed)
            state.advance_index()
        else:
            if playing and idx >= n - 1:
                state.set_playing(False)
            rospy.sleep(0.05)


def main() -> None:
    """Run replay node: load experiment, publish poses to ROS."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
    )
    args = _parse_args()
    experiment_dir = args.experiment
    rate = args.rate
    interactive = args.interactive
    if rate <= 0:
        logger.error("--rate must be positive.")
        sys.exit(1)
    rate = max(SPEED_MIN, min(SPEED_MAX, rate))

    try:
        data = load_experiment(experiment_dir)
    except (ValueError, OSError) as e:
        logger.error("Failed to load experiment: %s", e)
        sys.exit(1)

    metadata = data.get("metadata") or {}
    drone_logs = data.get("drone_logs") or []

    if not drone_logs:
        logger.error("No drone CSV files in experiment dir: %s", experiment_dir)
        sys.exit(1)

    num_drones = len(drone_logs)
    steps = list(iter_steps(loaded=data, align="t"))
    if not steps:
        logger.error("No steps from iter_steps.")
        sys.exit(1)

    rospy.init_node("replay_rviz", anonymous=True)

    pose_pubs: Dict[int, Any] = {}
    for i in range(1, num_drones + 1):
        pose_pubs[i] = rospy.Publisher(
            f"/swarm/drone_{i}/pose", PoseStamped, queue_size=10
        )
    meta_pub = rospy.Publisher("/swarm/metadata", String, queue_size=1)

    def shutdown(_signum: Optional[int] = None, _frame: Any = None) -> None:
        rospy.signal_shutdown("SIGINT or shutdown")

    signal.signal(signal.SIGINT, shutdown)

    rospy.loginfo(
        "Replay: %d drones, %d steps, rate=%.2f, interactive=%s",
        num_drones,
        len(steps),
        rate,
        interactive,
    )

    if interactive:
        state = PlaybackState(num_steps=len(steps), initial_speed=rate)
        _run_interactive(
            steps,
            pose_pubs,
            meta_pub,
            metadata,
            num_drones,
            state,
            use_keyboard=sys.stdin.isatty(),
        )
    else:
        meta_pub.publish(String(data=json.dumps(metadata)))
        _run_linear(steps, pose_pubs, num_drones, rate)


if __name__ == "__main__":
    main()

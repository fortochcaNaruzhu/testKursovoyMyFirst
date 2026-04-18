#!/usr/bin/env python3
"""
Graphical control for replay: play, pause, speed, seek.

Publishes to ROS topics /replay/play, /replay/pause, /replay/seek, /replay/speed.
Replay must be started with --interactive so it subscribes to these topics.
Uses only stdlib and rospy; reads experiment metadata and CSV files directly (no dependency on replay.csv_loader). No imports from core/ or scenarios/.
"""

import argparse
import json
import logging
import os
import sys
from typing import Optional, Tuple

# Ensure project root is on path when run as script
_script_dir = os.path.dirname(os.path.abspath(__file__))
_replay_parent = os.path.dirname(_script_dir)
if _replay_parent not in sys.path:
    sys.path.insert(0, _replay_parent)

try:
    import rospy
    from std_msgs.msg import Empty, Float32, Float64
except ImportError as e:
    print(
        "replay_gui.py requires ROS and rospy. Install ROS (e.g. Noetic) and source setup.bash.\n"
        "Error: " + str(e),
        file=sys.stderr,
    )
    sys.exit(1)

logger = logging.getLogger(__name__)

# Speed limits (match replay playback_controller)
SPEED_MIN = 0.25
SPEED_MAX = 4.0


def _read_experiment_time_range(experiment_path: str) -> Tuple[float, float]:
    """Read min and max time from experiment CSV for seek range.

    Args:
        experiment_path: Path to experiment directory (metadata.json + drone_*.csv).

    Returns:
        (t_min, t_max) in seconds. (0.0, 0.0) if no data or error.
    """
    metadata_path = os.path.join(experiment_path, "metadata.json")
    if not os.path.isfile(metadata_path):
        return (0.0, 0.0)
    try:
        with open(metadata_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
    except (json.JSONDecodeError, OSError):
        return (0.0, 0.0)
    num_drones = meta.get("num_drones", 0)
    if not isinstance(num_drones, (int, float)) or int(num_drones) < 1:
        return (0.0, 0.0)
    n = int(num_drones)
    t_min, t_max = float("inf"), float("-inf")
    for i in range(1, n + 1):
        path = os.path.join(experiment_path, f"drone_{i}.csv")
        if not os.path.isfile(path):
            continue
        with open(path, "r", encoding="utf-8") as f:
            f.readline()  # skip CSV header
            lines = [ln.strip() for ln in f if ln.strip()]
        if not lines:
            continue
        for line in lines:
            parts = line.split(",")
            if len(parts) >= 1:
                try:
                    t = float(parts[0])
                    t_min = min(t_min, t)
                    t_max = max(t_max, t)
                except ValueError:
                    pass
    if t_min == float("inf"):
        t_min = 0.0
    if t_max == float("-inf"):
        t_max = 0.0
    return (t_min, t_max)


def _run_gui(
    experiment_path: Optional[str],
    pub_play: rospy.Publisher,
    pub_pause: rospy.Publisher,
    pub_seek: rospy.Publisher,
    pub_speed: rospy.Publisher,
) -> None:
    """Run tkinter window and publish to replay topics on user action.

    Args:
        experiment_path: Path to experiment directory for seek range, or None.
        pub_play: ROS publisher for /replay/play.
        pub_pause: ROS publisher for /replay/pause.
        pub_seek: ROS publisher for /replay/seek (Float64 time).
        pub_speed: ROS publisher for /replay/speed (Float32).
    """
    import tkinter as tk
    from tkinter import ttk

    t_min, t_max = 0.0, 0.0
    if experiment_path and os.path.isdir(experiment_path):
        t_min, t_max = _read_experiment_time_range(experiment_path)
        if t_max <= t_min:
            t_max = t_min + 1.0

    root = tk.Tk()
    root.title("Replay control")
    root.geometry("380x180")
    root.resizable(True, False)

    speed_var = tk.DoubleVar(value=1.0)

    def on_play() -> None:
        pub_play.publish(Empty())
        logger.debug("Published /replay/play")

    def on_pause() -> None:
        pub_pause.publish(Empty())
        logger.debug("Published /replay/pause")

    def on_speed_change(value: str) -> None:
        try:
            v = float(value)
            v = max(SPEED_MIN, min(SPEED_MAX, v))
            pub_speed.publish(Float32(data=v))
            logger.debug("Published /replay/speed %.2f", v)
        except ValueError:
            pass

    def on_seek(value: str) -> None:
        try:
            t = float(value)
            pub_seek.publish(Float64(data=t))
            logger.debug("Published /replay/seek %.2f", t)
        except ValueError:
            pass

    row = 0
    btn_frame = ttk.Frame(root, padding=5)
    btn_frame.grid(row=row, column=0, sticky="ew")
    ttk.Button(btn_frame, text="Play", command=on_play).pack(side=tk.LEFT, padx=4)
    ttk.Button(btn_frame, text="Pause", command=on_pause).pack(side=tk.LEFT, padx=4)
    row += 1

    speed_frame = ttk.LabelFrame(root, text="Speed (0.25 – 4.0)", padding=5)
    speed_frame.grid(row=row, column=0, sticky="ew", padx=5, pady=5)
    speed_scale = ttk.Scale(
        speed_frame,
        from_=SPEED_MIN,
        to=SPEED_MAX,
        variable=speed_var,
        orient=tk.HORIZONTAL,
        command=on_speed_change,
    )
    speed_scale.pack(fill=tk.X, expand=True, padx=4)
    row += 1

    if t_max > t_min:
        seek_frame = ttk.LabelFrame(root, text="Seek (time, s)", padding=5)
        seek_frame.grid(row=row, column=0, sticky="ew", padx=5, pady=5)
        seek_var = tk.DoubleVar(value=t_min)
        seek_slider = ttk.Scale(
            seek_frame,
            from_=t_min,
            to=t_max,
            variable=seek_var,
            orient=tk.HORIZONTAL,
            command=on_seek,
        )
        seek_slider.pack(fill=tk.X, expand=True, padx=4)
        ttk.Label(seek_frame, text=f"{t_min:.1f} – {t_max:.1f} s").pack(anchor=tk.W)
        row += 1
    else:
        ttk.Label(root, text="No experiment path or no CSV times for seek.").grid(
            row=row, column=0, padx=5, pady=5
        )
        row += 1

    root.columnconfigure(0, weight=1)
    rospy.loginfo(
        "Replay GUI: start replay with --interactive first; controls publish to /replay/*"
    )
    root.mainloop()


def main() -> None:
    """Parse args, init ROS node, create publishers, run GUI."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
    )
    parser = argparse.ArgumentParser(
        description="Replay control GUI: play, pause, speed, seek (publishes to /replay/*). "
        "Start replay_rviz.py with --interactive first."
    )
    parser.add_argument(
        "--experiment",
        type=str,
        default=None,
        metavar="PATH",
        help="Experiment directory path (for seek range from metadata/CSV).",
    )
    parser.add_argument(
        "--gui-only",
        action="store_true",
        help="Skip loading experiment (no seek slider range).",
    )
    args = parser.parse_args()
    experiment_path = None if args.gui_only else args.experiment

    rospy.init_node("replay_gui", anonymous=True)
    pub_play = rospy.Publisher("/replay/play", Empty, queue_size=1)
    pub_pause = rospy.Publisher("/replay/pause", Empty, queue_size=1)
    pub_seek = rospy.Publisher("/replay/seek", Float64, queue_size=1)
    pub_speed = rospy.Publisher("/replay/speed", Float32, queue_size=1)
    rospy.sleep(0.5)

    _run_gui(experiment_path, pub_play, pub_pause, pub_seek, pub_speed)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
2D replay of experiment from CSV logs (read-only).

Uses replay/csv_loader to load experiment data; no imports from core/ or scenarios/.
Plot uses same NED convention as live visualizer: X = North, Y = East.

Run from project root:
  python visualizer/replay_2d.py --experiment experiments/2026-02-27_22-02-24
  python visualizer/replay_2d.py --experiment experiments/<timestamp> --rate 2.0 --trail 50
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Ensure project root is on path when run as script
_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPT_DIR.parent
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

try:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

try:
    from replay.csv_loader import load_experiment, iter_steps
    HAS_REPLAY = True
except ImportError:
    HAS_REPLAY = False

logger = logging.getLogger(__name__)

PALETTE = [
    "#2563eb", "#dc2626", "#16a34a", "#ca8a04", "#9333ea",
    "#0d9488", "#ea580c", "#4f46e5", "#059669", "#b91c1c",
]


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="2D replay of experiment from CSV (NED: X=North, Y=East)"
    )
    parser.add_argument(
        "--experiment", "-e", required=True,
        help="Path to experiment directory (e.g. experiments/2026-02-27_22-02-24)",
    )
    parser.add_argument(
        "--rate", "-r", type=float, default=1.0,
        help="Playback speed multiplier (default 1.0)",
    )
    parser.add_argument(
        "--trail", type=int, default=30,
        help="Trail length in steps (0 = current only)",
    )
    parser.add_argument(
        "--interval", type=float, default=0.05,
        help="Plot update interval in seconds",
    )
    return parser.parse_args()


def run_replay(
    experiment_dir: str,
    rate: float = 1.0,
    trail: int = 30,
    interval: float = 0.05,
) -> None:
    """Run 2D matplotlib replay from CSV data.

    Args:
        experiment_dir: Path to experiment folder.
        rate: Playback speed multiplier.
        trail: Number of past points per drone (0 = current only).
        interval: Minimum time between frame updates (seconds).
    """
    if not HAS_MATPLOTLIB:
        logger.error("matplotlib required. Install: pip install matplotlib")
        sys.exit(1)
    if not HAS_REPLAY:
        logger.error("replay.csv_loader not found. Run from project root.")
        sys.exit(1)

    loaded = load_experiment(experiment_dir)
    metadata = loaded["metadata"]
    num_drones = metadata["num_drones"]
    duration_sec = metadata.get("duration_sec", 0)

    # Pre-load all steps for animation (align by time)
    steps_list: List[Tuple[float, List[Optional[Dict[str, Any]]]]]] = list(
        iter_steps(loaded=loaded, align="t")
    )
    if not steps_list:
        logger.error("No steps in experiment.")
        sys.exit(1)

    # Compute average step dt for playback rate
    dts = []
    for i in range(1, min(len(steps_list), 100)):
        dts.append(steps_list[i][0] - steps_list[i - 1][0])
    avg_dt = sum(dts) / len(dts) if dts else 0.1
    frame_interval_ms = max(int(interval * 1000), int((avg_dt / rate) * 1000), 50)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlabel("X (m, North, NED)")
    ax.set_ylabel("Y (m, East, NED)")
    ax.set_title(f"2D Replay: {metadata.get('scenario', 'experiment')} — {num_drones} drones")
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="box")

    trail_n = max(0, trail)
    lines: Dict[int, Any] = {}
    points: Dict[int, Any] = {}
    histories: Dict[int, List[Tuple[float, float]]] = {
        i: [] for i in range(1, num_drones + 1)
    }
    t0: List[float] = [0.0]

    for i in range(1, num_drones + 1):
        color = PALETTE[(i - 1) % len(PALETTE)]
        label = "Drone 1 (leader)" if i == 1 else ("Drone 2 (follower)" if i == 2 else f"Drone {i}")
        lines[i], = ax.plot([], [], "-", color=color, linewidth=1.5, alpha=0.6, label=label)
        points[i], = ax.plot([], [], "o", color=color, markersize=10)

    step_index = [0]

    def init() -> List[Any]:
        return list(lines.values()) + list(points.values())

    def animate(_: Any) -> List[Any]:
        idx = step_index[0]
        if idx >= len(steps_list):
            return list(lines.values()) + list(points.values())

        t, row_list = steps_list[idx]

        for drone_idx, row in enumerate(row_list):
            did = drone_idx + 1
            if row is None:
                continue
            x, y = float(row["x"]), float(row["y"])
            h = histories[did]
            h.append((x, y))
            if trail_n > 0 and len(h) > trail_n:
                h[:] = h[-trail_n:]
            elif trail_n == 0 and h:
                h[:] = [h[-1]]

            if h:
                xs = [p[0] for p in h]
                ys = [p[1] for p in h]
                lines[did].set_data(xs, ys)
                points[did].set_data([xs[-1]], [ys[-1]])

        if lines:
            ax.legend(
                [lines[d] for d in sorted(lines.keys())],
                [lines[d].get_label() for d in sorted(lines.keys())],
                loc="upper left", fontsize=9,
            )

        all_x, all_y = [], []
        for h in histories.values():
            if h:
                pts = h[-trail_n:] if trail_n > 0 else h[-1:]
                all_x.extend(p[0] for p in pts)
                all_y.extend(p[1] for p in pts)
        if all_x and all_y:
            margin = 2.0
            x_min, x_max = min(all_x) - margin, max(all_x) + margin
            y_min, y_max = min(all_y) - margin, max(all_y) + margin
            rx = x_max - x_min
            ry = y_max - y_min
            r = max(rx, ry, 1.0) / 2
            cx = (x_min + x_max) / 2
            cy = (y_min + y_max) / 2
            ax.set_xlim(cx - r, cx + r)
            ax.set_ylim(cy - r, cy + r)

        step_index[0] += 1

        # Throttle by real time so playback rate is respected
        if idx == 0:
            t0[0] = time.monotonic()
        if idx + 1 < len(steps_list):
            next_t = steps_list[idx + 1][0]
            dt = (next_t - t) / rate if rate > 0 else 0
            target = t0[0] + (next_t - steps_list[0][0]) / rate
            sleep = target - time.monotonic()
            if sleep > 0:
                time.sleep(min(sleep, interval))

        return list(lines.values()) + list(points.values())

    anim = animation.FuncAnimation(
        fig, animate, init_func=init,
        interval=int(interval * 1000),
        blit=False,
        cache_frame_data=False,
    )
    plt.tight_layout()
    logger.info(
        "Replay: %s drones, %d steps, rate=%.1f (interval=%.0f ms). Close window to exit.",
        num_drones, len(steps_list), rate, frame_interval_ms,
    )
    plt.show()


def main() -> None:
    """Entry point."""
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    run_replay(
        experiment_dir=args.experiment,
        rate=args.rate,
        trail=args.trail,
        interval=args.interval,
    )


if __name__ == "__main__":
    main()

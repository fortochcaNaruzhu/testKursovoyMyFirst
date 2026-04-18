#!/usr/bin/env python3
"""
Plot experiment CSV logs: position (x, y, z) vs time and 2D trajectory.

Reads experiment directory (metadata.json + drone_*.csv), optionally downsamples
to reduce duplicate rows (same position logged many times when loop runs faster
than SITL position updates).

Usage:
    python scripts/plot_experiment_csv.py experiments/2026-02-28_18-35-11
    python scripts/plot_experiment_csv.py experiments/2026-02-28_18-35-11 --downsample 50
"""

import argparse
import os
import sys
from pathlib import Path

# Add project root for imports
_project_root = Path(__file__).resolve().parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

# Use non-interactive backend when no display (e.g. SSH, CI)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_drone_csv(path: str) -> np.ndarray:
    """Load one drone CSV; return array (t, x, y, z, rx, ry, rz, hasCollision)."""
    data = []
    with open(path, "r", encoding="utf-8") as f:
        header = f.readline()
        if "t,x,y,z" not in header:
            raise ValueError(f"Unexpected CSV header: {header.strip()}")
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) != 8:
                continue
            row = [float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]),
                   float(parts[4]), float(parts[5]), float(parts[6]), int(parts[7])]
            data.append(row)
    return np.array(data) if data else np.zeros((0, 8))


def downsample_unique_positions(arr: np.ndarray, min_dt: float = 0.0) -> np.ndarray:
    """Keep rows where (x,y,z) changed or time advanced by at least min_dt."""
    if len(arr) == 0:
        return arr
    out = [arr[0]]
    last_xyzt = (arr[0, 1], arr[0, 2], arr[0, 3], arr[0, 0])
    for i in range(1, len(arr)):
        t, x, y, z = arr[i, 0], arr[i, 1], arr[i, 2], arr[i, 3]
        if (x, y, z) != (last_xyzt[0], last_xyzt[1], last_xyzt[2]) or (min_dt > 0 and t - last_xyzt[3] >= min_dt):
            out.append(arr[i])
            last_xyzt = (x, y, z, t)
    return np.array(out)


def load_position_errors(path: str) -> tuple[np.ndarray, list[int]]:
    """Load position_errors.csv; return (array with t in col 0 and errors in cols 1..), list of drone ids."""
    t_list = []
    err_cols: list[list[float]] = []
    drone_ids: list[int] = []
    with open(path, "r", encoding="utf-8") as f:
        header = f.readline().strip()
        if not header.startswith("t,"):
            return np.zeros((0, 1)), []
        parts = header.split(",")
        for i in range(1, len(parts)):
            if parts[i].startswith("drone_"):
                try:
                    drone_ids.append(int(parts[i].replace("drone_", "")))
                except ValueError:
                    pass
        for line in f:
            line = line.strip()
            if not line:
                continue
            vals = line.split(",")
            if len(vals) != 1 + len(drone_ids):
                continue
            try:
                t_list.append(float(vals[0]))
                err_cols.append([float(vals[j + 1]) for j in range(len(drone_ids))])
            except ValueError:
                continue
    if not t_list:
        return np.zeros((0, 1 + len(drone_ids))), drone_ids
    arr = np.column_stack([np.array(t_list)] + [np.array([r[j] for r in err_cols]) for j in range(len(drone_ids))])
    return arr, drone_ids


def plot_experiment(experiment_dir: str, downsample_hz: float = 0, out_path: str | None = None) -> None:
    """
    Plot all drone CSVs: position vs time; bottom = trajectory (x,y) or position error vs time if position_errors.csv exists.

    Args:
        experiment_dir: Path to experiment folder (metadata.json + drone_*.csv).
        downsample_hz: If > 0, show only one point per 1/downsample_hz seconds (reduces duplicate rows).
        out_path: If set, save figure to this path; otherwise show interactively.
    """
    exp = Path(experiment_dir)
    if not exp.is_dir():
        raise FileNotFoundError(f"Not a directory: {experiment_dir}")

    csv_files = sorted(exp.glob("drone_*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No drone_*.csv in {experiment_dir}")

    position_errors_path = exp / "position_errors.csv"
    has_errors = position_errors_path.is_file()
    err_arr, err_drone_ids = load_position_errors(str(position_errors_path)) if has_errors else (np.zeros((0, 1)), [])

    min_dt = (1.0 / downsample_hz) if downsample_hz > 0 else 0.0
    all_data = {}
    for p in csv_files:
        did = int(p.stem.split("_")[1])
        arr = load_drone_csv(str(p))
        arr_plot = downsample_unique_positions(arr, min_dt)
        all_data[did] = (arr, arr_plot)

    n_drones = len(all_data)
    n_rows = 2
    fig, axes = plt.subplots(n_rows, 1, figsize=(12, 8), sharex=True)
    if n_rows == 1:
        axes = [axes]
    ax_time = axes[0]
    ax_bottom = axes[1]

    colors = plt.cm.tab10(np.linspace(0, 1, max(n_drones, 1)))

    for idx, (did, (full_arr, plot_arr)) in enumerate(sorted(all_data.items())):
        c = colors[idx % len(colors)]
        label = f"drone_{did}"
        if len(plot_arr) == 0:
            continue
        t, x, y, z = plot_arr[:, 0], plot_arr[:, 1], plot_arr[:, 2], plot_arr[:, 3]
        ax_time.plot(t, x, "-", color=c, alpha=0.8, label=f"{label} x")
        ax_time.plot(t, y, "--", color=c, alpha=0.6, label=f"{label} y")
        ax_time.plot(t, z, ":", color=c, alpha=0.8, label=f"{label} z")
        if not has_errors:
            ax_bottom.plot(x, y, "-", color=c, label=label)

    ax_time.set_ylabel("Position (m)")
    ax_time.set_title("Position vs time (x, y, z)")
    ax_time.legend(loc="upper right", fontsize=7, ncol=2)
    ax_time.grid(True, alpha=0.3)

    if has_errors and len(err_arr) > 0 and len(err_drone_ids) > 0:
        for idx, did in enumerate(err_drone_ids):
            c = colors[(did - 1) % len(colors)]
            t_err = err_arr[:, 0]
            e = err_arr[:, idx + 1]
            ax_bottom.plot(t_err, e, "-", color=c, alpha=0.8, label=f"drone_{did}")
        ax_bottom.set_xlabel("Time (s)")
        ax_bottom.set_ylabel("Position error (m)")
        ax_bottom.set_title("Position control error vs time")
        ax_bottom.legend(loc="upper right", fontsize=8)
        ax_bottom.grid(True, alpha=0.3)
    else:
        ax_bottom.set_xlabel("x (m)")
        ax_bottom.set_ylabel("y (m)")
        ax_bottom.set_title("Trajectory (x, y)")
        ax_bottom.legend(loc="upper right", fontsize=8)
        ax_bottom.set_aspect("equal", adjustable="datalim")
        ax_bottom.grid(True, alpha=0.3)
    plt.tight_layout()

    if out_path:
        plt.savefig(out_path, dpi=150, bbox_inches="tight")
        print(f"Saved: {out_path}")
    else:
        plt.show()
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot experiment CSV: position vs time and 2D trajectory."
    )
    parser.add_argument(
        "experiment_dir",
        type=str,
        help="Path to experiment directory (e.g. experiments/2026-02-28_18-35-11)",
    )
    parser.add_argument(
        "--downsample",
        type=float,
        default=0,
        help="Downsample to ~N Hz (only keep one point per 1/N s) to reduce duplicate rows; 0 = only unique (x,y,z)",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=None,
        help="Save figure to this path instead of showing",
    )
    args = parser.parse_args()
    plot_experiment(args.experiment_dir, downsample_hz=args.downsample, out_path=args.out)


if __name__ == "__main__":
    main()

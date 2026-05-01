#!/usr/bin/env python3
"""
Plot experiment CSV quality graphs (drone_*.csv) for N drones.

Experiment CSV format:
  t,x,y,z,rx,ry,rz,hasCollision

Outputs (into <experiment_dir>/plots by default):
  - experiment_compare__dt_over_time_<Nd>.png  (grid)
  - experiment_compare__dt_overlay.png         (overlay)
  - experiment_compare__dt_hist_<Nd>.png       (grid hist)
  - experiment_compare__dpos_over_time_<Nd>.png(grid of |Δpos| over time)
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt


def _read_drone_csv(path: str) -> Tuple[List[float], List[Tuple[float, float, float]]]:
    ts: List[float] = []
    pos: List[Tuple[float, float, float]] = []
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            ts.append(float(row["t"]))
            pos.append((float(row["x"]), float(row["y"]), float(row["z"])))
    return ts, pos


def _dt(ts: List[float]) -> List[float]:
    return [b - a for a, b in zip(ts, ts[1:]) if (b - a) >= 0]


def _dpos(pos: List[Tuple[float, float, float]]) -> List[float]:
    out: List[float] = []
    for (x0, y0, z0), (x1, y1, z1) in zip(pos, pos[1:]):
        dx = x1 - x0
        dy = y1 - y0
        dz = z1 - z0
        out.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    return out


def _grid(n: int) -> Tuple[int, int]:
    ncols = min(4, max(1, n))
    nrows = (n + ncols - 1) // ncols
    return nrows, ncols


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot experiment drone_*.csv quality graphs.")
    ap.add_argument(
        "experiment_dir",
        nargs="?",
        default="../test_logov/exp_antena_logic_mavlink_8d_50hz_01_50sec",
        help="Experiment directory containing drone_*.csv and metadata.json.",
    )
    ap.add_argument(
        "--out-dir",
        default=None,
        help="Output directory (default: <experiment_dir>/plots).",
    )
    args = ap.parse_args()

    exp_dir = os.path.abspath(os.path.expanduser(args.experiment_dir))
    if not os.path.isdir(exp_dir):
        raise SystemExit(f"Experiment dir not found: {exp_dir}")

    # Discover drone_*.csv
    drone_files = []
    for name in os.listdir(exp_dir):
        if name.startswith("drone_") and name.endswith(".csv"):
            drone_files.append(name)
    if not drone_files:
        raise SystemExit("No drone_*.csv found in experiment dir.")

    def _did(name: str) -> int:
        try:
            return int(name.replace("drone_", "").replace(".csv", ""))
        except Exception:
            return 10**9

    drone_files.sort(key=_did)
    ids = [_did(n) for n in drone_files]
    paths = [os.path.join(exp_dir, n) for n in drone_files]
    n = len(paths)

    out_dir = (
        os.path.abspath(os.path.expanduser(args.out_dir))
        if args.out_dir
        else os.path.join(exp_dir, "plots")
    )
    os.makedirs(out_dir, exist_ok=True)

    series = []
    for did, p in zip(ids, paths):
        ts, pos = _read_drone_csv(p)
        series.append(
            {
                "id": did,
                "ts": ts,
                "dt": _dt(ts),
                "dpos": _dpos(pos),
            }
        )

    nrows, ncols = _grid(n)
    tag = f"{n}d"

    # 1) dt over time grid
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 3.1 * nrows))
    axl = axes.ravel() if hasattr(axes, "ravel") else [axes]
    for i, s in enumerate(series):
        ax = axl[i]
        dt = s["dt"]
        ts = s["ts"]
        if dt and len(ts) >= 2:
            ax.plot(ts[1 : 1 + len(dt)], dt, linewidth=1.0)
        ax.grid(True, alpha=0.35)
        ax.set_title(f"d{s['id']}")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("Δt between rows (s)")
    for j in range(n, len(axl)):
        axl[j].axis("off")
    fig.suptitle("Experiment CSV: Δt over time (per drone)")
    p1 = os.path.join(out_dir, f"experiment_compare__dt_over_time_{tag}.png")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(p1, dpi=160)
    plt.close(fig)

    # 2) dt overlay
    plt.figure(figsize=(12.5, 4.8))
    for s in series:
        dt = s["dt"]
        ts = s["ts"]
        if dt and len(ts) >= 2:
            plt.plot(ts[1 : 1 + len(dt)], dt, linewidth=1.0, label=f"d{s['id']}")
    plt.grid(True, alpha=0.35)
    plt.xlabel("t (s)")
    plt.ylabel("Δt between rows (s)")
    plt.title(f"Experiment CSV: Δt overlay ({n} drones)")
    plt.legend(ncol=min(8, n), fontsize=8)
    p2 = os.path.join(out_dir, "experiment_compare__dt_overlay.png")
    plt.tight_layout()
    plt.savefig(p2, dpi=160)
    plt.close()

    # 3) dt hist grid
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 3.1 * nrows))
    axl = axes.ravel() if hasattr(axes, "ravel") else [axes]
    for i, s in enumerate(series):
        ax = axl[i]
        dt = s["dt"]
        if dt:
            ax.hist(dt, bins=50, alpha=0.9)
        ax.grid(True, alpha=0.35)
        ax.set_title(f"d{s['id']}")
        ax.set_xlabel("Δt (s)")
        ax.set_ylabel("count")
    for j in range(n, len(axl)):
        axl[j].axis("off")
    fig.suptitle("Experiment CSV: Δt histogram (per drone)")
    p3 = os.path.join(out_dir, f"experiment_compare__dt_hist_{tag}.png")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(p3, dpi=160)
    plt.close(fig)

    # 4) |Δpos| over time grid
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 3.1 * nrows))
    axl = axes.ravel() if hasattr(axes, "ravel") else [axes]
    for i, s in enumerate(series):
        ax = axl[i]
        dp = s["dpos"]
        ts = s["ts"]
        if dp and len(ts) >= 2:
            ax.plot(ts[1 : 1 + len(dp)], dp, linewidth=1.0)
        ax.grid(True, alpha=0.35)
        ax.set_title(f"d{s['id']}")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("|Δpos| (m)")
    for j in range(n, len(axl)):
        axl[j].axis("off")
    fig.suptitle("Experiment CSV: |Δpos| over time (per drone)")
    p4 = os.path.join(out_dir, f"experiment_compare__dpos_over_time_{tag}.png")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(p4, dpi=160)
    plt.close(fig)

    print("Wrote experiment plots:")
    for p in (p1, p2, p3, p4):
        print(" -", p)


if __name__ == "__main__":
    main()


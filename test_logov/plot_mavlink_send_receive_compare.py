#!/usr/bin/env python3
"""
Compare send-timing vs receive-timing for MAVLink LOCAL_POSITION_NED logs.

Input CSV is produced by mavlink_position_rate_logger.py:
  wall_time, dt_wall, time_boot_ms, x, y, z, vx, vy, vz

Key idea:
  - dt_wall      : inter-arrival time on the logger machine (receive side)
  - dt_boot_s    : (Δtime_boot_ms)/1000 measured inside SITL/autopilot message (send side)
If dt_boot_s grows to 0.25s, SITL is sending slowly (or stream interval changed).
If dt_boot_s stays ~0.02s but dt_wall grows, delivery/processing is stalling.

Outputs per CSV:
  - <stem>__dt_wall_vs_dt_boot.png
"""

from __future__ import annotations

import argparse
import csv
import os
from typing import List, Tuple

import matplotlib.pyplot as plt


def _read(path: str) -> Tuple[List[float], List[float], List[int]]:
    wall: List[float] = []
    dt_wall: List[float] = []
    boot_ms: List[int] = []
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            wall.append(float(row["wall_time"]))
            boot_ms.append(int(row.get("time_boot_ms", "0") or 0))
            sdt = (row.get("dt_wall") or "").strip()
            if sdt:
                try:
                    dt_wall.append(float(sdt))
                except ValueError:
                    pass
    return wall, dt_wall, boot_ms


def _diff_int(xs: List[int]) -> List[int]:
    out: List[int] = []
    for a, b in zip(xs, xs[1:]):
        d = b - a
        if d >= 0:
            out.append(d)
    return out


def plot_one(csv_path: str, out_dir: str) -> str:
    wall, dt_wall, boot_ms = _read(csv_path)
    if len(wall) < 3:
        raise RuntimeError(f"Not enough rows in {csv_path}")

    t_rel = [t - wall[0] for t in wall]

    boot_d_ms = _diff_int(boot_ms)
    dt_boot_s = [d / 1000.0 for d in boot_d_ms]

    # Align arrays:
    # - dt_wall corresponds to wall[1:] (except first row has empty dt_wall)
    # - dt_boot_s corresponds to boot_ms[1:]
    n = min(len(dt_wall), len(dt_boot_s), max(0, len(t_rel) - 1))
    x = t_rel[1 : 1 + n]
    y_wall = dt_wall[:n]
    y_boot = dt_boot_s[:n]

    # Difference and drift
    y_diff = [a - b for a, b in zip(y_wall, y_boot)]
    # Drift between receive clock and sender boot clock (relative), per message.
    boot_rel_s = [(b - boot_ms[0]) / 1000.0 for b in boot_ms]
    drift = [(tw - t_rel[0]) - tb for tw, tb in zip(t_rel, boot_rel_s)]

    stem = os.path.splitext(os.path.basename(csv_path))[0]
    out_path = os.path.join(out_dir, f"{stem}__dt_wall_vs_dt_boot.png")

    fig, axes = plt.subplots(3, 1, figsize=(12.5, 9.0), sharex=True)

    ax = axes[0]
    ax.plot(x, y_wall, linewidth=1.0, label="dt_wall (receive)")
    ax.plot(x, y_boot, linewidth=1.0, label="dt_boot_s (send, from time_boot_ms)")
    ax.grid(True, alpha=0.35)
    ax.set_ylabel("Δt (s)")
    ax.set_title(f"{stem}: inter-arrival (receive) vs inter-send (boot_ms)")
    ax.legend()

    ax = axes[1]
    ax.plot(x, y_diff, linewidth=1.0, color="#d33682")
    ax.grid(True, alpha=0.35)
    ax.set_ylabel("dt_wall - dt_boot (s)")
    ax.set_title("Difference per step (positive = receive slower than send)")

    ax = axes[2]
    ax.plot(t_rel, drift, linewidth=1.0, color="#268bd2")
    ax.grid(True, alpha=0.35)
    ax.set_xlabel("t_rel (s) = wall_time - wall_time[0]")
    ax.set_ylabel("drift (s)")
    ax.set_title("(wall_time_rel - boot_time_rel): drift/queueing indicator")

    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    return out_path


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot dt_wall vs time_boot_ms timing comparison.")
    ap.add_argument("csv", nargs="?", default=None, help="Path to mavlink_local_position_ned_port*.csv")
    ap.add_argument(
        "--experiment-dir",
        default=None,
        help="If set, process all mavlink_local_position_ned_port*.csv in this directory (or its mavlink_raw/).",
    )
    ap.add_argument("--out-dir", default=None, help="Output directory for PNGs.")
    args = ap.parse_args()

    if args.experiment_dir:
        exp = os.path.abspath(os.path.expanduser(args.experiment_dir))
        base = exp
        mr = os.path.join(exp, "mavlink_raw")
        if os.path.isdir(mr):
            base = mr
        out_dir = (
            os.path.abspath(os.path.expanduser(args.out_dir))
            if args.out_dir
            else os.path.join(exp, "timing_compare")
        )
        os.makedirs(out_dir, exist_ok=True)
        csvs = sorted(
            [
                os.path.join(base, n)
                for n in os.listdir(base)
                if n.startswith("mavlink_local_position_ned_port") and n.endswith(".csv")
            ]
        )
        if not csvs:
            raise SystemExit(f"No mavlink_local_position_ned_port*.csv found in {base}")
        for p in csvs:
            plot_one(p, out_dir)
        print(f"Wrote {len(csvs)} timing-compare plots into {out_dir}")
        return

    if not args.csv:
        raise SystemExit("Provide either CSV path or --experiment-dir")

    csv_path = os.path.abspath(os.path.expanduser(args.csv))
    if not os.path.exists(csv_path):
        raise SystemExit(f"CSV not found: {csv_path}")
    out_dir = (
        os.path.abspath(os.path.expanduser(args.out_dir))
        if args.out_dir
        else os.path.dirname(csv_path)
    )
    os.makedirs(out_dir, exist_ok=True)
    out = plot_one(csv_path, out_dir)
    print("Wrote", out)


if __name__ == "__main__":
    main()


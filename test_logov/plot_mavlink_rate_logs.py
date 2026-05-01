#!/usr/bin/env python3
"""
Plot evidence graphs for MAVLink LOCAL_POSITION_NED arrival timing.

Input CSV format produced by mavlink_position_rate_logger.py:
  wall_time, dt_wall, time_boot_ms, x, y, z, vx, vy, vz

Outputs PNG files next to the CSV (or in --out-dir).
"""

from __future__ import annotations

import argparse
import csv
import os
from collections import Counter
from typing import List, Tuple

import matplotlib.pyplot as plt


def _read_csv(path: str) -> Tuple[List[float], List[float], List[int]]:
    wall: List[float] = []
    dt: List[float] = []
    boot: List[int] = []
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            wall.append(float(row["wall_time"]))
            b = int(row.get("time_boot_ms", "0") or 0)
            boot.append(b)
            sdt = (row.get("dt_wall") or "").strip()
            if sdt:
                try:
                    dt.append(float(sdt))
                except ValueError:
                    pass
    return wall, dt, boot


def _quantiles(xs: List[float], ps=(0.5, 0.9, 0.95, 0.99)) -> dict:
    if not xs:
        return {}
    s = sorted(xs)
    out = {}
    for p in ps:
        i = int(p * (len(s) - 1))
        out[p] = s[i]
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot MAVLink LOCAL_POSITION_NED arrival timing graphs.")
    ap.add_argument(
        "csv",
        nargs="?",
        default="test_logov/mavlink_local_position_ned_port14551.csv",
        help="Input CSV path (default: test_logov/mavlink_local_position_ned_port14551.csv).",
    )
    ap.add_argument(
        "--out-dir",
        default=None,
        help="Output directory for PNGs (default: same directory as input CSV).",
    )
    args = ap.parse_args()

    in_path = os.path.abspath(os.path.expanduser(args.csv))
    if not os.path.exists(in_path):
        raise SystemExit(f"CSV not found: {in_path}")

    out_dir = (
        os.path.abspath(os.path.expanduser(args.out_dir))
        if args.out_dir
        else os.path.dirname(in_path)
    )
    os.makedirs(out_dir, exist_ok=True)

    wall, dt_wall, boot_ms = _read_csv(in_path)
    if len(wall) < 2:
        raise SystemExit("Not enough rows to plot.")

    # Convert wall_time to relative seconds for plotting.
    t0 = wall[0]
    t_rel = [t - t0 for t in wall]

    # boot diffs
    boot_d = [b1 - b0 for b0, b1 in zip(boot_ms, boot_ms[1:]) if (b1 - b0) >= 0]
    boot_counts = Counter(boot_d)

    # Summary numbers (used in plot titles)
    q = _quantiles(dt_wall)
    dt_mean = (sum(dt_wall) / len(dt_wall)) if dt_wall else 0.0
    hz_eff = (1.0 / dt_mean) if dt_mean > 0 else 0.0

    stem = os.path.splitext(os.path.basename(in_path))[0]

    # 1) dt_wall over time
    plt.figure(figsize=(10, 4.5))
    if dt_wall:
        # dt_wall has one fewer sample; align with t_rel[1:]
        plt.plot(t_rel[1 : 1 + len(dt_wall)], dt_wall, linewidth=1.0)
    plt.grid(True, alpha=0.35)
    plt.xlabel("t_rel (s)")
    plt.ylabel("dt_wall between messages (s)")
    title = f"{stem}: dt_wall over time | mean={dt_mean:.4f}s (~{hz_eff:.1f}Hz)"
    if q:
        title += f" | p95={q.get(0.95, 0):.3f}s p99={q.get(0.99, 0):.3f}s"
    plt.title(title)
    p1 = os.path.join(out_dir, f"{stem}__dt_wall_over_time.png")
    plt.tight_layout()
    plt.savefig(p1, dpi=160)
    plt.close()

    # 2) Histogram of dt_wall
    plt.figure(figsize=(8.5, 4.5))
    if dt_wall:
        plt.hist(dt_wall, bins=60, color="#2aa198", alpha=0.9)
    plt.grid(True, alpha=0.35)
    plt.xlabel("dt_wall (s)")
    plt.ylabel("count")
    title = f"{stem}: dt_wall histogram"
    if q:
        title += f" | median={q.get(0.5, 0):.3f}s p95={q.get(0.95, 0):.3f}s"
    plt.title(title)
    p2 = os.path.join(out_dir, f"{stem}__dt_wall_hist.png")
    plt.tight_layout()
    plt.savefig(p2, dpi=160)
    plt.close()

    # 3) time_boot_ms diff bar chart (most common)
    common = boot_counts.most_common(12)
    xs = [str(k) for k, _ in common]
    ys = [v for _, v in common]
    plt.figure(figsize=(8.5, 4.5))
    plt.bar(xs, ys, color="#268bd2", alpha=0.9)
    plt.grid(True, axis="y", alpha=0.35)
    plt.xlabel("Δtime_boot_ms between samples (ms)")
    plt.ylabel("count")
    plt.title(f"{stem}: most common time_boot_ms diffs (evidence of 33/34ms + gaps)")
    p3 = os.path.join(out_dir, f"{stem}__time_boot_ms_diffs.png")
    plt.tight_layout()
    plt.savefig(p3, dpi=160)
    plt.close()

    # 4) Cumulative message count over time
    plt.figure(figsize=(10, 4.5))
    plt.plot(t_rel, list(range(1, len(t_rel) + 1)), linewidth=1.2)
    plt.grid(True, alpha=0.35)
    plt.xlabel("t_rel (s)")
    plt.ylabel("cumulative messages")
    plt.title(f"{stem}: cumulative LOCAL_POSITION_NED messages (flat slope = stalls)")
    p4 = os.path.join(out_dir, f"{stem}__cumulative_messages.png")
    plt.tight_layout()
    plt.savefig(p4, dpi=160)
    plt.close()

    print("Wrote plots:")
    for p in (p1, p2, p3, p4):
        print(" -", p)


if __name__ == "__main__":
    main()


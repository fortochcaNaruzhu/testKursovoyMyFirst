#!/usr/bin/env python3
"""
Plot effective rates for position telemetry (LOCAL_POSITION_NED or SIM_STATE) vs ATTITUDE from combined mavlink_msgs.csv.

Input: CSV produced by mavlink_position_rate_logger.py with --include-attitude.
Columns:
  wall_time,msg_type,dt_wall_any,dt_wall_type,time_boot_ms,... (pos/att fields)

Outputs:
  - <stem>__dt_wall_type_over_time.png   (2 lines)
  - <stem>__dt_boot_over_time.png        (2 lines, using time_boot_ms diffs per type)
  - <stem>__boot_diffs_hist.png          (histogram per type)
"""

from __future__ import annotations

import argparse
import csv
import os
from collections import Counter
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt


def main() -> None:
    ap = argparse.ArgumentParser(description="Compare POSITION vs ATTITUDE timing from combined log.")
    ap.add_argument("csv", help="Path to combined mavlink_msgs.csv")
    ap.add_argument("--out-dir", default=None, help="Output directory for PNGs (default: alongside CSV).")
    args = ap.parse_args()

    p = os.path.abspath(os.path.expanduser(args.csv))
    if not os.path.exists(p):
        raise SystemExit(f"CSV not found: {p}")
    out_dir = (
        os.path.abspath(os.path.expanduser(args.out_dir))
        if args.out_dir
        else os.path.dirname(p)
    )
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(p))[0]

    # Roll LOCAL_POSITION_NED and SIM_STATE into one "POSITION" series vs ATTITUDE.
    t_rel_by: Dict[str, List[float]] = {"POSITION": [], "ATTITUDE": []}
    dt_wall_by: Dict[str, List[float]] = {"POSITION": [], "ATTITUDE": []}
    boot_by: Dict[str, List[int]] = {"POSITION": [], "ATTITUDE": []}
    _POS_TYPES = frozenset({"LOCAL_POSITION_NED", "SIM_STATE"})

    wall0 = None
    with open(p, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            wt = float(row["wall_time"])
            if wall0 is None:
                wall0 = wt
            mtype = row["msg_type"]
            if mtype in _POS_TYPES:
                bucket = "POSITION"
            elif mtype == "ATTITUDE":
                bucket = "ATTITUDE"
            else:
                continue
            t_rel_by[bucket].append(wt - wall0)
            sdt = (row.get("dt_wall_type") or "").strip()
            dt_wall_by[bucket].append(float(sdt) if sdt else float("nan"))
            try:
                boot_by[bucket].append(int(float(row.get("time_boot_ms") or 0)))
            except ValueError:
                boot_by[bucket].append(0)

    # Plot dt_wall_type over time (receive)
    plt.figure(figsize=(12.5, 4.8))
    for mtype, ts in t_rel_by.items():
        if len(ts) < 2:
            continue
        plt.plot(ts, dt_wall_by[mtype], linewidth=1.0, label=f"{mtype} dt_wall_type")
    plt.grid(True, alpha=0.35)
    plt.xlabel("t_rel (s)")
    plt.ylabel("dt_wall_type (s)")
    plt.title(f"{stem}: receive-side dt_wall_type (POSITION vs ATTITUDE)")
    plt.legend()
    out1 = os.path.join(out_dir, f"{stem}__dt_wall_type_over_time.png")
    plt.tight_layout()
    plt.savefig(out1, dpi=160)
    plt.close()

    # Compute boot diffs per type and plot over time (send)
    plt.figure(figsize=(12.5, 4.8))
    boot_hist: Dict[str, Counter] = {}
    for mtype, boots in boot_by.items():
        if len(boots) < 2:
            continue
        diffs = [b1 - b0 for b0, b1 in zip(boots, boots[1:]) if (b1 - b0) >= 0]
        boot_hist[mtype] = Counter(diffs)
        # align to timestamps (use t_rel of samples 1..)
        ts = t_rel_by[mtype][1 : 1 + len(diffs)]
        ys = [d / 1000.0 for d in diffs]
        plt.plot(ts, ys, linewidth=1.0, label=f"{mtype} Δboot (s)")
    plt.grid(True, alpha=0.35)
    plt.xlabel("t_rel (s)")
    plt.ylabel("Δtime_boot_ms (s)")
    plt.title(f"{stem}: send-side Δtime_boot_ms (POSITION vs ATTITUDE)")
    plt.legend()
    out2 = os.path.join(out_dir, f"{stem}__dt_boot_over_time.png")
    plt.tight_layout()
    plt.savefig(out2, dpi=160)
    plt.close()

    # Histograms of boot diffs (ms)
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.2), sharey=False)
    for ax, mtype in zip(axes, ["POSITION", "ATTITUDE"]):
        c = boot_hist.get(mtype, Counter())
        common = c.most_common(20)
        xs = [str(k) for k, _ in common]
        ys = [v for _, v in common]
        ax.bar(xs, ys, alpha=0.9)
        ax.grid(True, axis="y", alpha=0.35)
        ax.set_title(mtype)
        ax.set_xlabel("Δtime_boot_ms (ms)")
        ax.set_ylabel("count")
    fig.suptitle(f"{stem}: most common time_boot_ms diffs (send-side)")
    out3 = os.path.join(out_dir, f"{stem}__boot_diffs_hist.png")
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(out3, dpi=160)
    plt.close(fig)

    print("Wrote plots:")
    print(" -", out1)
    print(" -", out2)
    print(" -", out3)


if __name__ == "__main__":
    main()


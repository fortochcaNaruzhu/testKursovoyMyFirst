#!/usr/bin/env python3
"""
Compare MAVLink position telemetry arrival timing across multiple drones on one figure.

Reads CSVs produced by mavlink_position_rate_logger.py:
  wall_time, dt_wall_any, time_boot_ms, msg_type, …

Outputs:
  - one PNG with dt_wall-over-time subplots (auto grid)
  - one PNG with overlaid dt_wall series (N lines)
  - one PNG with boot_ms diff bar charts (auto grid)
"""

from __future__ import annotations

import argparse
import csv
import os
from collections import Counter
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt


def _read_one(path: str) -> Dict[str, List]:
    wall: List[float] = []
    dt: List[float] = []
    boot: List[int] = []
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        fields = r.fieldnames or ()
        has_msg_type = "msg_type" in fields
        for row in r:
            if has_msg_type:
                mt = (row.get("msg_type") or "").strip()
                if mt and mt not in ("LOCAL_POSITION_NED", "SIM_STATE"):
                    continue
            wall.append(float(row["wall_time"]))
            try:
                boot.append(int(float(row.get("time_boot_ms") or 0)))
            except ValueError:
                boot.append(0)
            sdt = (row.get("dt_wall") or row.get("dt_wall_any") or "").strip()
            if sdt:
                try:
                    dt.append(float(sdt))
                except ValueError:
                    pass
    return {"wall": wall, "dt": dt, "boot": boot}


def _ensure_paths(paths: List[str]) -> List[str]:
    out = []
    for p in paths:
        ap = os.path.abspath(os.path.expanduser(p))
        if not os.path.exists(ap):
            raise SystemExit(f"CSV not found: {ap}")
        out.append(ap)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot combined MAVLink timing graphs for multiple drones.")
    ap.add_argument(
        "--csv",
        nargs="+",
        default=[
            "../test_logov/mavlink_local_position_ned_port14551.csv",
            "../test_logov/mavlink_local_position_ned_port14561.csv",
            "../test_logov/mavlink_local_position_ned_port14571.csv",
            "../test_logov/mavlink_local_position_ned_port14581.csv",
        ],
        help="Input CSV paths (4 files).",
    )
    ap.add_argument("--labels", nargs="+", default=None, help="Labels for each CSV (same count).")
    ap.add_argument("--out-dir", default="../test_logov", help="Output directory for PNGs.")
    args = ap.parse_args()

    paths = _ensure_paths(list(args.csv))
    n = len(paths)
    if n < 2:
        raise SystemExit(f"Expected at least 2 CSV files, got {n}")

    labels = args.labels
    if labels is None:
        # Default labels from filenames (e.g. port14551)
        labels = [os.path.basename(p).replace("mavlink_local_position_ned_", "").replace(".csv", "") for p in paths]
    if len(labels) != len(paths):
        raise SystemExit("--labels must match number of --csv files")

    out_dir = os.path.abspath(os.path.expanduser(args.out_dir))
    os.makedirs(out_dir, exist_ok=True)

    data = [_read_one(p) for p in paths]

    # Common time origin for overlay: use each series t_rel (wall - wall[0]) independently.
    rel = []
    for d in data:
        w = d["wall"]
        t0 = w[0] if w else 0.0
        rel.append([t - t0 for t in w])

    # Determine subplot grid (aim for ~4 columns).
    ncols = min(4, n)
    nrows = (n + ncols - 1) // ncols

    # 1) dt_wall over time (grid)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 3.1 * nrows), sharex=False, sharey=False)
    axes_list = axes.ravel() if hasattr(axes, "ravel") else [axes]
    for i, (lab, d) in enumerate(zip(labels, data)):
        ax = axes_list[i]
        dt = d["dt"]
        tr = rel[i]
        if dt:
            ax.plot(tr[1 : 1 + len(dt)], dt, linewidth=1.0)
        ax.grid(True, alpha=0.35)
        ax.set_title(lab)
        ax.set_xlabel("t_rel (s)")
        ax.set_ylabel("dt_wall (s)")
    # Hide any unused axes
    for j in range(n, len(axes_list)):
        axes_list[j].axis("off")
    fig.suptitle("Position telemetry arrival dt_wall over time (per drone)")
    p1 = os.path.join(out_dir, f"mavlink_compare__dt_wall_over_time_{n}d.png")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(p1, dpi=160)
    plt.close(fig)

    # 2) Overlay dt_wall (N lines) on one plot
    plt.figure(figsize=(12.5, 4.8))
    for i, (lab, d) in enumerate(zip(labels, data)):
        dt = d["dt"]
        tr = rel[i]
        if not dt:
            continue
        plt.plot(tr[1 : 1 + len(dt)], dt, linewidth=1.0, label=lab)
    plt.grid(True, alpha=0.35)
    plt.xlabel("t_rel (s)")
    plt.ylabel("dt_wall (s)")
    plt.title(f"Position telemetry dt_wall overlay ({n} drones)")
    plt.legend()
    p2 = os.path.join(out_dir, "mavlink_compare__dt_wall_overlay.png")
    plt.tight_layout()
    plt.savefig(p2, dpi=160)
    plt.close()

    # 3) boot_ms diffs (grid)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4.2 * ncols, 3.1 * nrows))
    axes_list = axes.ravel() if hasattr(axes, "ravel") else [axes]
    for i, (lab, d) in enumerate(zip(labels, data)):
        ax = axes_list[i]
        b = d["boot"]
        dif = [b1 - b0 for b0, b1 in zip(b, b[1:]) if (b1 - b0) >= 0]
        c = Counter(dif)
        common = c.most_common(8)
        xs = [str(k) for k, _ in common]
        ys = [v for _, v in common]
        ax.bar(xs, ys, alpha=0.9)
        ax.grid(True, axis="y", alpha=0.35)
        ax.set_title(lab)
        ax.set_xlabel("Δtime_boot_ms (ms)")
        ax.set_ylabel("count")
    for j in range(n, len(axes_list)):
        axes_list[j].axis("off")
    fig.suptitle("Most common time_boot_ms diffs (per drone)")
    p3 = os.path.join(out_dir, f"mavlink_compare__time_boot_ms_diffs_{n}d.png")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(p3, dpi=160)
    plt.close(fig)

    print("Wrote comparison plots:")
    print(" -", p1)
    print(" -", p2)
    print(" -", p3)


if __name__ == "__main__":
    main()


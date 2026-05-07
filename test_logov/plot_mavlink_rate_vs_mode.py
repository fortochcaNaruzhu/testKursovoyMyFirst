#!/usr/bin/env python3
"""
Plot SITL-side position telemetry effective rate (LOCAL_POSITION_NED or SIM_STATE; Δtime_boot_ms) vs wall-clock time,
with background bands for flight mode (GUIDED / POSHOLD / …).

Input: CSV from mavlink_position_rate_logger.py (msg_type, wall_time, time_boot_ms, mode, …).
"""

from __future__ import annotations

import argparse
import csv
import os
import re
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np


def _mode_color(mode: str) -> Tuple[float, float, float, float]:
    m = (mode or "").strip().upper()
    if m == "GUIDED":
        return (0.35, 0.55, 0.95, 0.35)
    if m == "POSHOLD":
        return (0.45, 0.85, 0.45, 0.35)
    if m == "STABILIZE":
        return (0.75, 0.75, 0.78, 0.35)
    return (0.95, 0.75, 0.35, 0.35)


def _load_mode_segments(path: str) -> List[Tuple[float, float, str]]:
    """Return [(t0_rel, t1_rel, mode), …] from any row where mode changes (wall_time axis)."""
    rows: List[Tuple[float, str]] = []
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                wt = float(row["wall_time"])
            except (KeyError, ValueError):
                continue
            mode = str(row.get("mode") or "").strip() or "?"
            rows.append((wt, mode))
    if not rows:
        return []
    rows.sort(key=lambda x: x[0])
    t0 = rows[0][0]
    segments: List[Tuple[float, float, str]] = []
    cur_mode = rows[0][1]
    seg_start = rows[0][0]
    for wt, mode in rows[1:]:
        if mode != cur_mode:
            segments.append((seg_start - t0, wt - t0, cur_mode))
            seg_start = wt
            cur_mode = mode
    segments.append((seg_start - t0, rows[-1][0] - t0, cur_mode))
    return segments


def _load_position_rate(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Returns (wall_rel, hz_boot, mode_at_row) for position rows only (LOCAL_POSITION_NED or SIM_STATE).
    hz_boot = 1 / Δ(time_boot_ms) in seconds between consecutive position rows.
    """
    wt_list: List[float] = []
    boot_list: List[int] = []
    mode_list: List[str] = []
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            if row.get("msg_type") not in ("LOCAL_POSITION_NED", "SIM_STATE"):
                continue
            try:
                wt = float(row["wall_time"])
                boot = int(row.get("time_boot_ms") or 0)
            except (KeyError, ValueError):
                continue
            wt_list.append(wt)
            boot_list.append(boot)
            mode_list.append(str(row.get("mode") or "").strip() or "?")

    if len(wt_list) < 3:
        return np.array([]), np.array([]), np.array([])

    t0 = wt_list[0]
    wall_rel = np.array(wt_list, dtype=float) - t0
    boot = np.array(boot_list, dtype=np.int64)
    mode_arr = np.array(mode_list)

    dt_ms = np.diff(boot.astype(np.float64))
    with np.errstate(divide="ignore", invalid="ignore"):
        hz = 1000.0 / dt_ms
    hz = np.clip(np.nan_to_num(hz, nan=0.0, posinf=120.0, neginf=0.0), 0.0, 120.0)
    wall_mid = 0.5 * (wall_rel[1:] + wall_rel[:-1])
    mode_mid = mode_arr[1:]
    return wall_mid, hz, mode_mid


def _rolling_mean(x: np.ndarray, w: int) -> np.ndarray:
    if x.size == 0 or w <= 1:
        return x
    w = min(w, x.size)
    kernel = np.ones(w, dtype=float) / float(w)
    return np.convolve(x, kernel, mode="valid")


def _port_label(path: str) -> str:
    m = re.search(r"(\d{5})", os.path.basename(path))
    return m.group(1) if m else os.path.basename(path)


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot MAVLink position rate vs mode transitions.")
    ap.add_argument(
        "csv",
        nargs="+",
        help="One or more mavlink_msgs_port*.csv files.",
    )
    ap.add_argument(
        "--out",
        default=None,
        help="Output PNG path (default: first CSV dir / mavlink_rate_vs_mode.png).",
    )
    ap.add_argument("--smooth", type=int, default=25, help="Rolling mean window for Hz curve (default 25).")
    args = ap.parse_args()

    paths = [os.path.abspath(os.path.expanduser(p)) for p in args.csv]
    for p in paths:
        if not os.path.isfile(p):
            raise SystemExit(f"CSV not found: {p}")

    out_path = (
        os.path.abspath(os.path.expanduser(args.out))
        if args.out
        else os.path.join(os.path.dirname(paths[0]), "mavlink_rate_vs_mode.png")
    )
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    n = len(paths)
    ncols = 2 if n > 1 else 1
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 3.2 * nrows), squeeze=False, sharex=False)

    for idx, path in enumerate(paths):
        ax = axes[idx // ncols][idx % ncols]
        wall_mid, hz, _mode_mid = _load_position_rate(path)
        segments = _load_mode_segments(path)

        t_max = 1.0
        for s0, s1, mode in segments:
            rgba = _mode_color(mode)
            ax.axvspan(s0, s1, ymin=0, ymax=1, color=rgba[:3], alpha=rgba[3], lw=0, zorder=0)
            t_max = max(t_max, s1)

        if wall_mid.size > 0:
            t_max = max(t_max, float(np.max(wall_mid)))
            ax.scatter(wall_mid, hz, s=4, alpha=0.12, c="#222222", zorder=2)
            w = int(args.smooth)
            sm = _rolling_mean(hz, w)
            if sm.size > 0:
                off = (w - 1) // 2
                x_line = wall_mid[off : off + sm.size]
                ax.plot(x_line, sm, color="crimson", lw=1.4, zorder=3, label=f"pos rate (smoothed, w={w})")

        ax.set_title(f"UDP tap port {_port_label(path)} — position rate from Δtime_boot_ms")
        ax.set_xlabel("Time since first row in file (s)")
        ax.set_ylabel("Hz (SITL send cadence ~1/Δboot)")
        ax.set_xlim(0.0, t_max * 1.02)
        if hz.size:
            ymax = min(65.0, float(np.nanmax(hz)) * 1.15 + 5.0)
            ax.set_ylim(0.0, ymax)
        else:
            ax.set_ylim(0.0, 60.0)
        ax.grid(True, alpha=0.28)

        # Legend: modes present
        modes_seen = sorted({m for _, _, m in segments})
        handles = [
            plt.Rectangle((0, 0), 1, 1, fc=_mode_color(m)[:3], alpha=_mode_color(m)[3], label=m)
            for m in modes_seen
        ]
        if handles:
            ax.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.9)

    for j in range(len(paths), nrows * ncols):
        axes[j // ncols][j % ncols].set_visible(False)

    fig.suptitle("MAVLink position message rate vs mode (background)", fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Convert logs/two_drones_log_run_follower_*.csv (leader/follower columns in one file)
into an experiment directory for replay_rviz / replay_rviz2.

Prefer logs/two_drones_log_<run-id>_exchange_sync.csv when present: both positions are
from the same coordinate_exchange tick (smooth replay). The follower-only log can look
jerky on the leader because leader_* updates only when the exchange loop runs.

Input header expected:
  t,leader_x,leader_y,follower_x,follower_y,error_x,error_y,follower_vx,follower_vy,loop_dt

Output:
  metadata.json, drone_1.csv, drone_2.csv (standard t,x,y,z,rx,ry,rz,hasCollision)
  z and attitudes set to 0; hasCollision=0.

Usage:
  python scripts/follower_pair_csv_to_experiment.py logs/two_drones_log_run_follower_2.csv
  python replay/replay_rviz2.py --experiment logs/replay_from_follower_log
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path

CSV_HEADER = "t,x,y,z,rx,ry,rz,hasCollision"
EXPECTED_IN = (
    "t",
    "leader_x",
    "leader_y",
    "follower_x",
    "follower_y",
)


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

    p = argparse.ArgumentParser(description="Pair follower log → experiment dir for RViz replay.")
    p.add_argument("input_csv", type=str, help="Path to two_drones_log_run_follower_*.csv")
    p.add_argument(
        "-o",
        "--output-dir",
        type=str,
        default="",
        help="Output directory (default: logs/replay_from_<stem>)",
    )
    args = p.parse_args()

    in_path = Path(args.input_csv).resolve()
    if not in_path.is_file():
        print(f"File not found: {in_path}", file=sys.stderr)
        sys.exit(1)

    out_dir = Path(args.output_dir) if args.output_dir else root / "logs" / f"replay_from_{in_path.stem}"
    out_dir.mkdir(parents=True, exist_ok=True)

    with open(in_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            print("Empty CSV", file=sys.stderr)
            sys.exit(1)
        fields = [h.strip() for h in reader.fieldnames]
        for req in EXPECTED_IN:
            if req not in fields:
                print(f"Missing column {req!r}. Got: {fields}", file=sys.stderr)
                sys.exit(1)

        rows = list(reader)

    d1_path = out_dir / "drone_1.csv"
    d2_path = out_dir / "drone_2.csv"
    with open(d1_path, "w", encoding="utf-8", newline="") as f1, open(
        d2_path, "w", encoding="utf-8", newline=""
    ) as f2:
        f1.write(CSV_HEADER + "\n")
        f2.write(CSV_HEADER + "\n")
        for r in rows:
            try:
                t = float(r["t"])
                lx = float(r["leader_x"])
                ly = float(r["leader_y"])
                fx = float(r["follower_x"])
                fy = float(r["follower_y"])
            except (KeyError, ValueError) as e:
                print(f"Skip bad row: {e}", file=sys.stderr)
                continue
            f1.write(f"{t:.6f},{lx:.6f},{ly:.6f},0.0,0.0,0.0,0.0,0\n")
            f2.write(f"{t:.6f},{fx:.6f},{fy:.6f},0.0,0.0,0.0,0.0,0\n")

    meta = {
        "duration_sec": float(rows[-1]["t"]) if rows else 0.0,
        "collision_radius_m": 0.2,
        "num_drones": 2,
        "scenario": "converted_from_follower_pair_csv",
        "source_file": str(in_path),
    }
    with open(out_dir / "metadata.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    print(f"Wrote experiment: {out_dir}")
    print("Replay:")
    print(f"  source /opt/ros/jazzy/setup.bash")
    print(f"  cd {root}")
    print(f"  python3 replay/replay_rviz2.py --experiment {out_dir}")


if __name__ == "__main__":
    main()

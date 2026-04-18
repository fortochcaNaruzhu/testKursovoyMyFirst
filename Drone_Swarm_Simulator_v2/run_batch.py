#!/usr/bin/env python3
"""
Batch runner for Drone Swarm Simulator v2: runs multiple experiments in sequence.

Each run invokes launch_simulation.py in subprocess with SITL-only mode,
distinct experiment directory, and the same scenario/drones/duration.
Use from project root; no hardcoded absolute paths.
"""

import argparse
import json
import os
import subprocess
import sys
from typing import Any, Dict, List, Optional

project_root: str = os.path.dirname(os.path.abspath(__file__))
LAUNCH_SCRIPT: str = os.path.join(project_root, "launch_simulation.py")
EXPERIMENTS_DIR: str = os.path.join(project_root, "experiments")


def validate_run_output(experiment_dir: str, num_drones: int) -> Optional[str]:
    """Check that experiment dir contains metadata.json and drone_*.csv files.

    Args:
        experiment_dir: Path to experiment directory (relative or absolute).
        num_drones: Expected number of drones (for CSV count check).

    Returns:
        None if valid; otherwise error message string.
    """
    abs_path = (
        os.path.join(project_root, experiment_dir)
        if not os.path.isabs(experiment_dir)
        else experiment_dir
    )
    if not os.path.isdir(abs_path):
        return f"experiment dir not found: {abs_path}"
    meta_path = os.path.join(abs_path, "metadata.json")
    if not os.path.isfile(meta_path):
        return f"metadata.json missing in {experiment_dir}"
    csv_count = 0
    for i in range(1, num_drones + 1):
        if os.path.isfile(os.path.join(abs_path, f"drone_{i}.csv")):
            csv_count += 1
    if csv_count == 0:
        return f"no drone_*.csv files in {experiment_dir}"
    if csv_count < num_drones:
        return f"expected {num_drones} CSV files, found {csv_count} in {experiment_dir}"
    return None


def parse_args() -> argparse.Namespace:
    """Parse batch runner CLI arguments.

    Returns:
        Parsed namespace with runs, drones, duration, scenario, batch_id.
    """
    parser = argparse.ArgumentParser(
        description="Run multiple simulation experiments in sequence (SITL-only)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python run_batch.py --runs 3 --drones 2 --duration 60 --scenario leader_forward_back
  python run_batch.py --runs 2 --scenario square_pid --batch-id my_batch
        """,
    )
    parser.add_argument(
        "--runs",
        type=int,
        required=True,
        metavar="N",
        help="Number of runs to execute",
    )
    parser.add_argument(
        "--drones",
        type=int,
        default=2,
        metavar="D",
        help="Number of drones per run (default 2)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        metavar="T",
        help="Experiment duration in seconds per run (default 60)",
    )
    parser.add_argument(
        "--scenario",
        type=str,
        required=True,
        metavar="NAME",
        help="Scenario ID (e.g. leader_forward_back, square_pid)",
    )
    parser.add_argument(
        "--batch-id",
        type=str,
        default=None,
        help="Optional batch identifier; if set, dirs are experiments/batch_<id>_run_<run_id>",
    )
    return parser.parse_args()


def build_experiment_dir(run_id: int, batch_id: Optional[str]) -> str:
    """Return experiment directory name (relative to project root).

    Args:
        run_id: One-based run index.
        batch_id: Optional batch identifier; if set, name uses batch_<id>_run_<run_id>.

    Returns:
        Path relative to project root, e.g. experiments/exp_1 or experiments/batch_my_batch_run_1.
    """
    rel = f"exp_{run_id}" if not batch_id else f"batch_{batch_id}_run_{run_id}"
    return os.path.join("experiments", rel)


def run_single(
    run_id: int,
    scenario: str,
    drones: int,
    duration: float,
    batch_id: Optional[str],
) -> tuple[int, str]:
    """Invoke launch_simulation.py once for the given run and validate output.

    Args:
        run_id: One-based run index.
        scenario: Scenario ID (e.g. leader_forward_back, square_pid).
        drones: Number of drones per run.
        duration: Experiment duration in seconds.
        batch_id: Optional batch identifier for experiment directory naming.

    Returns:
        Tuple of (exit_code, experiment_dir). exit_code 0 only if subprocess
        and output validation (metadata.json + drone_*.csv) pass.
    """
    experiment_dir = build_experiment_dir(run_id, batch_id)
    cmd: List[str] = [
        sys.executable,
        LAUNCH_SCRIPT,
        "-s",
        "-c",
        scenario,
        "-n",
        str(drones),
        "--duration",
        str(duration),
        "--experiment-dir",
        experiment_dir,
    ]
    print(f"[Batch] Run {run_id}: experiment-dir={experiment_dir}")
    proc = subprocess.run(cmd, cwd=project_root)
    if proc.returncode != 0:
        return (proc.returncode, experiment_dir)
    err = validate_run_output(experiment_dir, drones)
    if err:
        print(f"[Batch] Run {run_id} validation failed: {err}", file=sys.stderr)
        return (1, experiment_dir)
    return (0, experiment_dir)


def main() -> None:
    """Parse config, run each experiment in sequence, validate output, write index.

    Runs launch_simulation.py in a subprocess for each run; validates presence of
    metadata.json and drone_*.csv. If --batch-id is set, writes batch_<id>_runs_index.json.
    Exits with code 1 if launcher not found or any run fails.
    """
    args = parse_args()
    if not os.path.isfile(LAUNCH_SCRIPT):
        print(f"Error: launcher not found: {LAUNCH_SCRIPT}")
        sys.exit(1)
    os.makedirs(EXPERIMENTS_DIR, exist_ok=True)
    failed = 0
    index: List[Dict[str, Any]] = []
    for run_id in range(1, args.runs + 1):
        code, experiment_dir = run_single(
            run_id=run_id,
            scenario=args.scenario,
            drones=args.drones,
            duration=args.duration,
            batch_id=args.batch_id,
        )
        if code != 0:
            failed += 1
            print(f"[Batch] Run {run_id} exited with code {code}")
        else:
            index.append(
                {
                    "run_id": run_id,
                    "experiment_dir": experiment_dir,
                    "num_drones": args.drones,
                    "duration_sec": args.duration,
                    "scenario": args.scenario,
                    "batch_id": args.batch_id,
                }
            )
    if args.batch_id and index:
        index_path = os.path.join(
            EXPERIMENTS_DIR, f"batch_{args.batch_id}_runs_index.json"
        )
        with open(index_path, "w", encoding="utf-8") as f:
            json.dump(index, f, indent=2)
        print(f"[Batch] Wrote index: {index_path}")
    if failed:
        print(f"[Batch] Completed with {failed} failed run(s)")
        sys.exit(1)
    print("[Batch] All runs completed successfully.")


if __name__ == "__main__":
    main()

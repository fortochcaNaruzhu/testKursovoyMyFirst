"""
Load experiment data from CSV logs and metadata.json.

Replay module is isolated: reads only CSV and metadata, does not import from core/ or scenarios/.
"""

import json
import os
from typing import Any, Dict, Generator, List, Optional, Tuple

# --- T1: Validation constants ---
REQUIRED_METADATA_KEYS = ("duration_sec", "collision_radius_m", "num_drones", "scenario")
CSV_HEADER = "t,x,y,z,rx,ry,rz,hasCollision"
EXPECTED_COLUMNS = 8


def _validate_metadata(metadata: Dict[str, Any]) -> None:
    """Validate that metadata contains all required keys.

    Args:
        metadata: Parsed metadata dict.

    Raises:
        ValueError: If any required key is missing.
    """
    missing = [k for k in REQUIRED_METADATA_KEYS if k not in metadata]
    if missing:
        raise ValueError(f"metadata.json missing required keys: {missing}")


def _validate_csv(path: str) -> None:
    """Validate CSV file: correct header and column count per row.

    Args:
        path: Path to the CSV file.

    Raises:
        FileNotFoundError: If file does not exist.
        ValueError: If header is wrong or any row has wrong column count.
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"CSV file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        first = f.readline()
        if not first.endswith("\n"):
            if first.strip() == "":
                raise ValueError(f"CSV {path}: empty file")
            header = first.strip()
        else:
            header = first.strip()
        if header != CSV_HEADER:
            raise ValueError(
                f"CSV {path}: invalid header (expected '{CSV_HEADER}', got '{header}')"
            )
        for i, line in enumerate(f, start=2):
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) != EXPECTED_COLUMNS:
                raise ValueError(
                    f"CSV {path} line {i}: expected {EXPECTED_COLUMNS} columns, got {len(parts)}"
                )


def load_experiment(experiment_dir: str) -> Dict[str, Any]:
    """Load and validate an experiment from disk.

    Expects experiment_dir to contain:
      - metadata.json with keys: duration_sec, collision_radius_m, num_drones, scenario
      - drone_1.csv, drone_2.csv, ... with header t,x,y,z,rx,ry,rz,hasCollision
        and exactly 8 columns per row.

    Args:
        experiment_dir: Path to the experiment directory.

    Returns:
        Dict with keys:
          - "metadata": Parsed metadata.json (dict).
          - "drone_logs": List of paths to drone CSV files (drone_1.csv, ...).
          - "experiment_dir": experiment_dir path (str).

    Raises:
        FileNotFoundError: If metadata.json or a required CSV is missing.
        ValueError: If metadata or any CSV fails validation.
    """
    metadata_path = os.path.join(experiment_dir, "metadata.json")
    if not os.path.isfile(metadata_path):
        raise FileNotFoundError(f"metadata.json not found in {experiment_dir}")
    with open(metadata_path, "r", encoding="utf-8") as f:
        metadata = json.load(f)
    _validate_metadata(metadata)
    num_drones = metadata["num_drones"]
    if not isinstance(num_drones, (int, float)) or int(num_drones) < 1:
        raise ValueError("metadata num_drones must be a positive integer")
    n = int(num_drones)
    drone_logs: List[str] = []
    for i in range(1, n + 1):
        path = os.path.join(experiment_dir, f"drone_{i}.csv")
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Required CSV not found: {path}")
        _validate_csv(path)
        drone_logs.append(path)
    return {
        "metadata": metadata,
        "drone_logs": drone_logs,
        "experiment_dir": experiment_dir,
    }


def _parse_row(line: str) -> Dict[str, Any]:
    """Parse one CSV data row into a dict with t,x,y,z,rx,ry,rz,hasCollision."""
    parts = line.strip().split(",")
    if len(parts) != EXPECTED_COLUMNS:
        raise ValueError(f"Row has {len(parts)} columns, expected {EXPECTED_COLUMNS}")
    return {
        "t": float(parts[0]),
        "x": float(parts[1]),
        "y": float(parts[2]),
        "z": float(parts[3]),
        "rx": float(parts[4]),
        "ry": float(parts[5]),
        "rz": float(parts[6]),
        "hasCollision": int(parts[7]),
    }


def _read_drone_csv(path: str) -> List[Dict[str, Any]]:
    """Read a single drone CSV into a list of row dicts (skip header)."""
    rows: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as f:
        header = f.readline().strip()
        if header != CSV_HEADER:
            raise ValueError(f"Invalid header in {path}: expected '{CSV_HEADER}'")
        for line in f:
            line = line.strip()
            if not line:
                continue
            rows.append(_parse_row(line))
    return rows


# Совпадение времени между drone_*.csv (float + разные моменты записи).
_TIME_BUCKET_DECIMALS = 6


def _time_bucket(t: float) -> float:
    return round(float(t), _TIME_BUCKET_DECIMALS)


def iter_steps(
    experiment_dir: Optional[str] = None,
    loaded: Optional[Dict[str, Any]] = None,
    align: str = "t",
) -> Generator[Tuple[float, List[Optional[Dict[str, Any]]]], None, None]:
    """Step-wise iteration over experiment data.

    For each time step yields (t, list_of_drone_dicts). Each dict has keys
    t, x, y, z, rx, ry, rz, hasCollision. List length equals num_drones;
    list[i] is the row for drone i+1 at this t, or None if that drone has
    no row for this timestamp.

    Args:
        experiment_dir: Path to experiment directory (used if loaded is None).
        loaded: Pre-loaded result of load_experiment() (if provided, experiment_dir ignored).
        align: "t" = merge rows whose t rounds to the same microsecond bucket, sort by t,
               then forward-fill so each step lists every drone that has appeared (no None
               gaps from float mismatch between files). "shortest" = by row index (shortest file).

    Yields:
        (t, list of per-drone dicts; dicts clone CSV row with t set to step time).

    Raises:
        ValueError: If neither experiment_dir nor loaded provided, or align is invalid.
    """
    if loaded is None:
        if not experiment_dir:
            raise ValueError("Either experiment_dir or loaded must be provided")
        loaded = load_experiment(experiment_dir)
    paths = loaded["drone_logs"]
    num_drones = len(paths)
    if num_drones == 0:
        return
    # Load all CSV data: rows_by_drone[i] = list of dicts for drone i
    rows_by_drone: List[List[Dict[str, Any]]] = []
    for path in paths:
        rows_by_drone.append(_read_drone_csv(path))
    if align == "shortest":
        n_steps = min(len(r) for r in rows_by_drone)
        for k in range(n_steps):
            t = rows_by_drone[0][k]["t"]
            step = [rows_by_drone[i][k] for i in range(num_drones)]
            yield (t, step)
        return
    if align != "t":
        raise ValueError("align must be 't' or 'shortest'")
    # Per bucket time: latest row per drone (prefer sample closest to bucket center).
    bucket_map: Dict[float, Dict[int, Dict[str, Any]]] = {}
    for drone_idx, rows in enumerate(rows_by_drone):
        for row in rows:
            tk = _time_bucket(row["t"])
            if tk not in bucket_map:
                bucket_map[tk] = {}
            prev = bucket_map[tk].get(drone_idx)
            if prev is None or abs(float(row["t"]) - tk) <= abs(float(prev["t"]) - tk):
                bucket_map[tk][drone_idx] = row
    last: List[Optional[Dict[str, Any]]] = [None] * num_drones
    for tk in sorted(bucket_map.keys()):
        for di, row in bucket_map[tk].items():
            last[di] = row
        step: List[Optional[Dict[str, Any]]] = []
        for i in range(num_drones):
            if last[i] is None:
                step.append(None)
            else:
                step.append({**last[i], "t": float(tk)})
        yield (float(tk), step)


class ReplaySteps:
    """Step-wise iteration over experiment data (class-based API).

    Use load_experiment() then ReplaySteps(loaded) and iterate to get
    (t, list of per-drone dicts) for each time step.
    """

    def __init__(
        self,
        experiment_dir: Optional[str] = None,
        loaded: Optional[Dict[str, Any]] = None,
        align: str = "t",
    ) -> None:
        """Initialize from experiment directory or pre-loaded data.

        Args:
            experiment_dir: Path to experiment directory (used if loaded is None).
            loaded: Pre-loaded result of load_experiment() (if provided, experiment_dir ignored).
            align: "t" or "shortest" (see iter_steps).
        """
        if loaded is None:
            if not experiment_dir:
                raise ValueError("Either experiment_dir or loaded must be provided")
            loaded = load_experiment(experiment_dir)
        self._loaded = loaded
        self._align = align

    def __iter__(
        self,
    ) -> Generator[Tuple[float, List[Optional[Dict[str, Any]]]], None, None]:
        """Yield (t, list of per-drone dicts) for each time step."""
        return iter_steps(loaded=self._loaded, align=self._align)

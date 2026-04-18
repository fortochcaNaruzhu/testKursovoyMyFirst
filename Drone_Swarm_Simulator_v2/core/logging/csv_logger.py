"""
CSV and metadata logging for experiments: per-drone CSV (t,x,y,z,rx,ry,rz,hasCollision)
and metadata.json.
"""

import json
import logging
import os
from typing import Any, Dict, Optional, TextIO

logger = logging.getLogger(__name__)

CSV_HEADER = "t,x,y,z,rx,ry,rz,hasCollision"


def write_row(
    file_handle: TextIO,
    drone_id: int,
    t: float,
    x: float,
    y: float,
    z: float,
    rx: float,
    ry: float,
    rz: float,
    has_collision: int,
) -> None:
    """
    Append one CSV row for a drone (one time step).

    Args:
        file_handle: Open file in text mode (or use a wrapper); will write one line.
        drone_id: Drone id (for logging only; not written to CSV).
        t: Time from start (seconds).
        x, y, z: NED position (m).
        rx, ry, rz: Euler angles roll, pitch, yaw (radians).
        has_collision: 0 or 1.
    """
    line = f"{t:.6f},{x:.6f},{y:.6f},{z:.6f},{rx:.6f},{ry:.6f},{rz:.6f},{has_collision}\n"
    file_handle.write(line)
    file_handle.flush()


def write_metadata(
    experiment_dir: str,
    duration_sec: float,
    collision_radius_m: float,
    num_drones: int,
    scenario: str,
    extra: Optional[Dict[str, Any]] = None,
) -> None:
    """
    Write metadata.json into the experiment directory.

    Args:
        experiment_dir: Path to experiment folder.
        duration_sec: Max experiment duration (0 = no limit).
        collision_radius_m: Drone sphere radius for collision detection (m).
        num_drones: Number of drones.
        scenario: Scenario name/id.
        extra: Optional extra key-value pairs to include in JSON.
    """
    data: Dict[str, Any] = {
        "duration_sec": duration_sec,
        "collision_radius_m": collision_radius_m,
        "num_drones": num_drones,
        "scenario": scenario,
    }
    if extra:
        data.update(extra)
    path = os.path.join(experiment_dir, "metadata.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    logger.info("Wrote metadata to %s", path)


class CSVLogger:
    """
    Helper to manage per-drone CSV files and optional metadata.

    Open one file per drone with header "t,x,y,z,rx,ry,rz,hasCollision";
    write_row adds one line per step. Call write_metadata once for the run.
    """

    def __init__(self, experiment_dir: str) -> None:
        """Initialize logger for an experiment directory.

        Args:
            experiment_dir: Directory for CSV files and metadata.json.
        """
        self.experiment_dir = experiment_dir
        self._files: Dict[int, TextIO] = {}

    def open_drone_log(self, drone_id: int) -> None:
        """Open drone_<id>.csv with standard header and store handle.

        Args:
            drone_id: Drone identifier; file will be named drone_<id>.csv.
        """
        path = os.path.join(self.experiment_dir, f"drone_{drone_id}.csv")
        f = open(path, "w", encoding="utf-8")
        f.write(CSV_HEADER + "\n")
        f.flush()
        self._files[drone_id] = f
        logger.debug("Opened log %s", path)

    def write_row(
        self,
        drone_id: int,
        t: float,
        x: float,
        y: float,
        z: float,
        rx: float,
        ry: float,
        rz: float,
        has_collision: int,
    ) -> None:
        """Append one row for the given drone.

        File for this drone must have been opened via open_drone_log.

        Args:
            drone_id: Drone identifier.
            t: Time from start (seconds).
            x, y, z: NED position (m).
            rx, ry, rz: Euler angles roll, pitch, yaw (radians).
            has_collision: 0 or 1.
        """
        if drone_id not in self._files:
            return
        write_row(
            self._files[drone_id],
            drone_id,
            t,
            x,
            y,
            z,
            rx,
            ry,
            rz,
            has_collision,
        )

    def close_all(self) -> None:
        """Close all open per-drone CSV file handles and clear internal state."""
        for did, f in self._files.items():
            try:
                f.close()
            except Exception:
                pass
        self._files.clear()
        logger.debug("Closed all experiment CSV files")

"""
Monitors drone position (LOCAL_POSITION_NED) and attitude (ATTITUDE) via MAVLinkWorker.

No dedicated threads: reads from the worker's thread-safe state cache.
One MAVLink connection is used only by the worker's single thread.
"""

import logging
from typing import Any, Dict

from core.mavlink.worker import MAVLinkWorker

logger = logging.getLogger(__name__)

_DEFAULT_POSITION: Dict[str, float] = {"x": 0.0, "y": 0.0, "z": 0.0}
_DEFAULT_ATTITUDE: Dict[str, float] = {"rx": 0.0, "ry": 0.0, "rz": 0.0}


class CoordsMonitor:
    """
    Reads NED position and Euler attitude from a MAVLinkWorker (thread-safe cache).

    Does not own any MAVLink connection or threads. All pymavlink I/O is done
    in the worker's single thread.
    """

    def __init__(self, worker: MAVLinkWorker) -> None:
        """
        Args:
            worker: MAVLinkWorker instance for this drone (get_position, get_attitude).
        """
        self._worker = worker

    def start(self) -> None:
        """No-op: worker thread is already running. Kept for API compatibility."""
        pass

    def stop(self) -> None:
        """No-op: worker is stopped by the owner. Kept for API compatibility."""
        pass

    def get_position(self) -> Dict[str, float]:
        """Return current NED position (x, y, z)."""
        pos = self._worker.get_position()
        if pos is None:
            return dict(_DEFAULT_POSITION)
        return {"x": pos["x"], "y": pos["y"], "z": pos["z"]}

    def get_attitude(self) -> Dict[str, float]:
        """Return current Euler angles (roll, pitch, yaw) in radians as rx, ry, rz."""
        att = self._worker.get_attitude()
        if att is None:
            return dict(_DEFAULT_ATTITUDE)
        return {"rx": att["rx"], "ry": att["ry"], "rz": att["rz"]}

    def get_distance_to(self, target_position: Dict[str, float]) -> float:
        """
        Distance in meters to the given target position (x, y, z).

        Args:
            target_position: Dict with keys 'x', 'y', 'z'.

        Returns:
            Distance in meters.
        """
        pos = self.get_position()
        dx = target_position["x"] - pos["x"]
        dy = target_position["y"] - pos["y"]
        dz = target_position["z"] - pos["z"]
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def get_relative_position(
        self, target_position: Dict[str, float]
    ) -> Dict[str, float]:
        """
        Relative position to target (target - self).

        Args:
            target_position: Dict with keys 'x', 'y', 'z'.

        Returns:
            Dict with 'x', 'y', 'z' relative offsets in meters.
        """
        pos = self.get_position()
        return {
            "x": target_position["x"] - pos["x"],
            "y": target_position["y"] - pos["y"],
            "z": target_position["z"] - pos["z"],
        }

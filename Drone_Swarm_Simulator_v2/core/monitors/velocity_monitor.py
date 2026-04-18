"""
Monitors drone velocity (vx, vy, vz) from MAVLinkWorker state.

Velocity is taken from LOCAL_POSITION_NED (vx, vy, vz) cached in the worker.
No dedicated thread: reads from the worker's thread-safe state.
"""

import logging
from typing import Any, Dict

from core.mavlink.worker import MAVLinkWorker

logger = logging.getLogger(__name__)

_DEFAULT_VELOCITY: Dict[str, float] = {"vx": 0.0, "vy": 0.0, "vz": 0.0}


class VelocityMonitor:
    """
    Returns NED velocity (vx, vy, vz) from the MAVLinkWorker position cache.

    Does not own any MAVLink connection or threads. The worker's single thread
    updates position (including vx, vy, vz from LOCAL_POSITION_NED).
    """

    def __init__(self, worker: MAVLinkWorker) -> None:
        """
        Args:
            worker: MAVLinkWorker instance (get_position returns x,y,z,vx,vy,vz).
        """
        self._worker = worker

    def start(self) -> None:
        """No-op: worker thread is already running. Kept for API compatibility."""
        pass

    def stop(self) -> None:
        """No-op: worker is stopped by the owner. Kept for API compatibility."""
        pass

    def get_velocity(self) -> Dict[str, float]:
        """Return current NED velocity (vx, vy, vz) in m/s."""
        pos = self._worker.get_position()
        if pos is None:
            return dict(_DEFAULT_VELOCITY)
        return {
            "vx": pos.get("vx", 0.0),
            "vy": pos.get("vy", 0.0),
            "vz": pos.get("vz", 0.0),
        }

"""
Shared DroneController: MAVLinkWorker, CoordsMonitor, VelocityMonitor, RC keepalive.

All pymavlink access goes through the worker (thread-safe). Scenarios import this
class and pass scenario-specific init_steps to initialize(). Algorithm-specific
logic (e.g. move_towards_with_pid, move_with_pid_braking) stays in scenarios.
"""

import os
import threading
import time
from typing import Any, Dict, List, Optional

from core.mavlink.utils import RC_NEUTRAL
from core.mavlink.worker import MAVLinkWorker
from core.monitors.coords_monitor import CoordsMonitor
from core.monitors.velocity_monitor import VelocityMonitor


class DroneController:
    """Controller for one drone: MAVLinkWorker, monitors, RC keepalive. Thread-safe."""

    def __init__(
        self,
        config: Dict[str, Any],
        logging_enabled: bool = False,
        log_dir: str = "logs",
    ) -> None:
        """Initialize controller with config and optional file logging.

        Args:
            config: Dict with 'id', 'udp_port', 'role'.
            logging_enabled: If True, write position logs to log_dir/drone_<id>_log.txt.
            log_dir: Directory for log files when logging_enabled is True.
        """
        self.config: Dict[str, Any] = config
        self.worker: Optional[MAVLinkWorker] = None
        self.coords_monitor: Optional[CoordsMonitor] = None
        self.velocity_monitor: Optional[VelocityMonitor] = None
        self.other_drones_positions: Dict[int, Dict[str, float]] = {}
        self.lock: threading.Lock = threading.Lock()
        self.logging_enabled: bool = logging_enabled
        self.log_dir: str = log_dir
        self.log_iter: int = 0
        self.logfile: Optional[Any] = None
        if self.logging_enabled:
            os.makedirs(self.log_dir, exist_ok=True)
            self.logfile = open(
                os.path.join(self.log_dir, f"drone_{self.config['id']}_log.txt"), "w"
            )
        self.last_rc_channels = {
            "roll": RC_NEUTRAL,
            "pitch": RC_NEUTRAL,
            "throttle": RC_NEUTRAL,
            "yaw": RC_NEUTRAL,
        }
        self.rc_channels_lock = threading.Lock()

    def connect(self) -> None:
        """Create MAVLinkWorker, start it, and create CoordsMonitor and VelocityMonitor."""
        conn_str = f'udp:127.0.0.1:{self.config["udp_port"]}'
        self.worker = MAVLinkWorker(conn_str, self.config["id"])
        self.worker.start()
        self.coords_monitor = CoordsMonitor(self.worker)
        self.velocity_monitor = VelocityMonitor(self.worker)

    def connect_with_heartbeat_timeout(self, heartbeat_timeout: Optional[float]) -> None:
        """
        Connect like connect(), but fail fast if no heartbeat is received.

        Args:
            heartbeat_timeout: Seconds to wait for heartbeat; None = wait indefinitely.
        """
        conn_str = f'udp:127.0.0.1:{self.config["udp_port"]}'
        self.worker = MAVLinkWorker(conn_str, self.config["id"])
        self.worker.start_with_heartbeat_timeout(heartbeat_timeout)
        self.coords_monitor = CoordsMonitor(self.worker)
        self.velocity_monitor = VelocityMonitor(self.worker)

    def initialize(self, init_steps: List[Dict[str, Any]]) -> bool:
        """Run init sequence (arm, takeoff, set mode, etc.) via worker.

        Args:
            init_steps: List of command dicts for worker.run_init_sequence()
                (e.g. set_mode, sleep, arm, takeoff, rc_override, request_position_stream).

        Returns:
            True if the MAVLink worker completed all steps within its timeout.
        """
        if self.worker is None:
            return False
        return bool(self.worker.run_init_sequence(init_steps))

    def start_rc_keepalive(self) -> None:
        """Start a daemon thread that periodically sends RC_OVERRIDE via worker."""
        def keepalive_loop() -> None:
            while self.worker is not None:
                try:
                    with self.rc_channels_lock:
                        roll = self.last_rc_channels["roll"]
                        pitch = self.last_rc_channels["pitch"]
                        throttle = self.last_rc_channels["throttle"]
                        yaw = self.last_rc_channels["yaw"]
                    if self.worker:
                        self.worker.send_rc_override(
                            roll, pitch, throttle, yaw, controller=self
                        )
                    time.sleep(0.2)
                except Exception:
                    break

        threading.Thread(target=keepalive_loop, daemon=True).start()

    def get_position(self) -> Dict[str, float]:
        """Return this drone's last known NED position (x, y, z).

        Returns:
            Dict with keys 'x', 'y', 'z' (meters in NED). Defaults to origin if unavailable.
        """
        if self.coords_monitor is None:
            if self.worker:
                pos = self.worker.get_position()
                if pos is not None:
                    return {"x": pos["x"], "y": pos["y"], "z": pos["z"]}
            return {"x": 0.0, "y": 0.0, "z": 0.0}
        return self.coords_monitor.get_position()

    def get_my_position(self) -> Dict[str, float]:
        """Return this drone's position; optionally log to file when logging_enabled.

        Returns:
            Dict with keys 'x', 'y', 'z' (NED position in meters).
        """
        pos = self.get_position()
        if self.logging_enabled and self.logfile:
            self.log_iter += 1
            if self.log_iter % 20 == 0:
                self.logfile.write(f"{pos}\n")
                self.logfile.flush()
        return pos

    def update_other_drone_position(
        self, drone_id: int, position: Dict[str, float]
    ) -> None:
        """Store another drone's position for formation/coordination."""
        with self.lock:
            self.other_drones_positions[drone_id] = position

    def get_other_drones_positions(self) -> Dict[int, Dict[str, float]]:
        """Return a copy of other drones' positions (thread-safe).

        Returns:
            Shallow copy of drone_id -> position dict; inner position dicts are copied.
        """
        with self.lock:
            return {k: dict(v) for k, v in self.other_drones_positions.items()}

    def stop(self) -> None:
        """Send neutral RC, command LAND via worker, stop monitors and worker."""
        if self.worker:
            self.worker.send_rc_override(
                RC_NEUTRAL,
                RC_NEUTRAL,
                RC_NEUTRAL,
                RC_NEUTRAL,
                controller=self,
            )
            self.worker.send_set_mode(9)  # LAND
        if self.coords_monitor:
            self.coords_monitor.stop()
        if self.velocity_monitor:
            self.velocity_monitor.stop()
        if self.logging_enabled and self.logfile:
            try:
                self.logfile.close()
            except Exception:
                pass
            self.logfile = None
        if self.worker:
            self.worker.stop()

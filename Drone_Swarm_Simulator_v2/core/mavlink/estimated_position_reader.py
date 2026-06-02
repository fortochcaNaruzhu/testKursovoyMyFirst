"""
Passive MAVLink estimated-position reader for SITL tap ports.

This reader subscribes only to ArduPilot-estimated pose messages and caches the
latest position/velocity per drone. It never sends control, arming, or mode
commands.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Iterable, Optional

from pymavlink import mavutil

from core.mavlink.geo_ned import ned_metres_from_home

logger = logging.getLogger(__name__)

BASE_HOME_LAT = 47.0
BASE_HOME_LON = 8.0
BASE_HOME_ALT = 0.0

_GLOBAL_POSITION_INT_MSG_ID = int(
    getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 33)
)
_LOCAL_POSITION_NED_MSG_ID = int(
    getattr(mavutil.mavlink, "MAVLINK_MSG_ID_LOCAL_POSITION_NED", 32)
)


class PassiveEstimatedPositionReader:
    """Read LOCAL_POSITION_NED or GLOBAL_POSITION_INT from dedicated UDP tap ports."""

    def __init__(
        self,
        *,
        drone_ids: Iterable[int],
        position_source: str = "local",
        port_base: int = 14651,
        port_step: int = 10,
        hz: float = 20.0,
        heartbeat_timeout_s: float = 12.0,
        home_y_offset_step_m: float = 2.0,
        base_home_lat: float = BASE_HOME_LAT,
        base_home_lon: float = BASE_HOME_LON,
        base_home_alt: float = BASE_HOME_ALT,
    ) -> None:
        source = str(position_source).strip().lower()
        if source not in {"local", "global"}:
            raise ValueError("position_source must be 'local' or 'global'")
        self.drone_ids = [int(did) for did in drone_ids]
        self.position_source = source
        self.port_base = int(port_base)
        self.port_step = int(port_step)
        self.hz = float(hz)
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.home_y_offset_step_m = float(home_y_offset_step_m)
        self.base_home_lat = float(base_home_lat)
        self.base_home_lon = float(base_home_lon)
        self.base_home_alt = float(base_home_alt)

        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []
        self._positions: Dict[int, Dict[str, float]] = {}
        self._running = False

    def start(self) -> None:
        """Start one daemon receiver thread per drone."""
        if self._running:
            return
        self._stop_event.clear()
        self._running = True
        for drone_id in self.drone_ids:
            port = self.port_base + (int(drone_id) - 1) * self.port_step
            thread = threading.Thread(
                target=self._run_drone,
                args=(int(drone_id), int(port)),
                daemon=True,
                name=f"estimated-position-drone-{drone_id}",
            )
            thread.start()
            self._threads.append(thread)
        logger.info(
            "Estimated-position reader started: source=%s port_base=%s hz=%.1f",
            self.position_source,
            self.port_base,
            self.hz,
        )

    def stop(self) -> None:
        """Stop receiver threads and clear running state."""
        self._stop_event.set()
        for thread in list(self._threads):
            thread.join(timeout=2.0)
        self._threads.clear()
        self._running = False

    def close(self) -> None:
        """Alias for stop(), matching logger-style lifecycle."""
        self.stop()

    def get_position(self, drone_id: int) -> Optional[Dict[str, float]]:
        """Return the latest common-frame estimated position for one drone."""
        with self._state_lock:
            pos = self._positions.get(int(drone_id))
            return dict(pos) if pos is not None else None

    def get_positions(self) -> Dict[int, Dict[str, float]]:
        """Return a copy of all latest common-frame estimated positions."""
        with self._state_lock:
            return {did: dict(pos) for did, pos in self._positions.items()}

    def _run_drone(self, drone_id: int, port: int) -> None:
        connection_string = f"udp:127.0.0.1:{port}"
        master: Any = None
        try:
            master = mavutil.mavlink_connection(connection_string)
            heartbeat = master.wait_heartbeat(timeout=self.heartbeat_timeout_s)
            if heartbeat is None:
                logger.warning(
                    "Drone %s: no heartbeat on estimated-position tap %s within %.1fs",
                    drone_id,
                    connection_string,
                    self.heartbeat_timeout_s,
                )
                return
            self._request_estimate_stream(master)
            self._receive_loop(drone_id, master)
        except Exception as exc:
            logger.warning("Drone %s: estimated-position reader stopped: %s", drone_id, exc)
        finally:
            try:
                if master is not None:
                    master.close()
            except Exception:
                pass

    def _request_estimate_stream(self, master: Any) -> None:
        interval_us = max(1, int(1_000_000.0 / max(0.1, self.hz)))
        msg_id = (
            _LOCAL_POSITION_NED_MSG_ID
            if self.position_source == "local"
            else _GLOBAL_POSITION_INT_MSG_ID
        )
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def _receive_loop(self, drone_id: int, master: Any) -> None:
        msg_type = (
            "LOCAL_POSITION_NED"
            if self.position_source == "local"
            else "GLOBAL_POSITION_INT"
        )
        while not self._stop_event.is_set():
            msg = master.recv_match(type=[msg_type], blocking=True, timeout=0.5)
            if msg is None or msg.get_type() == "BAD_DATA":
                continue
            pos = (
                self._position_from_local(drone_id, msg)
                if self.position_source == "local"
                else self._position_from_global(drone_id, msg)
            )
            with self._state_lock:
                self._positions[int(drone_id)] = pos

    def _position_from_local(self, drone_id: int, msg: Any) -> Dict[str, float]:
        time_boot_ms = getattr(msg, "time_boot_ms", None)
        return {
            "x": float(getattr(msg, "x", 0.0)),
            "y": float(getattr(msg, "y", 0.0)) + self._did_offset_y(drone_id),
            "z": float(getattr(msg, "z", 0.0)),
            "vx": float(getattr(msg, "vx", 0.0)),
            "vy": float(getattr(msg, "vy", 0.0)),
            "vz": float(getattr(msg, "vz", 0.0)),
            "time_boot_s": float(time_boot_ms) / 1000.0 if time_boot_ms is not None else 0.0,
            "receipt_time_s": time.time(),
        }

    def _position_from_global(self, drone_id: int, msg: Any) -> Dict[str, float]:
        lat_deg = float(getattr(msg, "lat", 0.0)) / 1.0e7
        lon_deg = float(getattr(msg, "lon", 0.0)) / 1.0e7
        rel_alt = getattr(msg, "relative_alt", None)
        if rel_alt is not None:
            z = -float(rel_alt) / 1000.0
        else:
            alt_m = float(getattr(msg, "alt", 0.0)) / 1000.0
            _x_unused, _y_unused, z = ned_metres_from_home(
                lat_deg,
                lon_deg,
                alt_m,
                self.base_home_lat,
                self.base_home_lon,
                self.base_home_alt,
            )
        x, y, _z_unused = ned_metres_from_home(
            lat_deg,
            lon_deg,
            self.base_home_alt - z,
            self.base_home_lat,
            self.base_home_lon,
            self.base_home_alt,
        )
        time_boot_ms = getattr(msg, "time_boot_ms", None)
        return {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "vx": float(getattr(msg, "vx", 0.0)) / 100.0,
            "vy": float(getattr(msg, "vy", 0.0)) / 100.0,
            "vz": float(getattr(msg, "vz", 0.0)) / 100.0,
            "time_boot_s": float(time_boot_ms) / 1000.0 if time_boot_ms is not None else 0.0,
            "receipt_time_s": time.time(),
        }

    def _did_offset_y(self, drone_id: int) -> float:
        return float(int(drone_id) - 1) * self.home_y_offset_step_m

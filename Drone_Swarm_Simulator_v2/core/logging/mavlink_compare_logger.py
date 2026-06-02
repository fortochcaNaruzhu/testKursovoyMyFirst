"""
Passive MAVLink comparison logger for SITL UDP tap ports.

The logger opens separate read connections from the control worker and only sends
MAV_CMD_SET_MESSAGE_INTERVAL requests for telemetry messages. It never sends
control, mode, arming, or RC commands.
"""

from __future__ import annotations

import csv
import logging
import os
import threading
import time
from typing import Any, Dict, Iterable, List, Optional, TextIO

from pymavlink import mavutil

from core.mavlink.geo_ned import sim_state_lat_lon_deg

logger = logging.getLogger(__name__)

_SIM_STATE_MSG_ID = int(getattr(mavutil.mavlink, "MAVLINK_MSG_ID_SIM_STATE", 108))
_GLOBAL_POSITION_INT_MSG_ID = int(
    getattr(mavutil.mavlink, "MAVLINK_MSG_ID_GLOBAL_POSITION_INT", 33)
)
_LOCAL_POSITION_NED_MSG_ID = int(
    getattr(mavutil.mavlink, "MAVLINK_MSG_ID_LOCAL_POSITION_NED", 32)
)

MAVLINK_COMPARE_HEADER = [
    "timestamp_unix_s",
    "t_rel_s",
    "drone_id",
    "msg_type",
    "msg_receipt_count",
    "sim_time_boot_ms",
    "sim_lat_deg",
    "sim_lon_deg",
    "sim_alt_m",
    "sim_roll_rad",
    "sim_pitch_rad",
    "sim_yaw_rad",
    "sim_vn_m_s",
    "sim_ve_m_s",
    "sim_vd_m_s",
    "sim_xacc",
    "sim_yacc",
    "sim_zacc",
    "gpi_time_boot_ms",
    "gpi_lat_deg",
    "gpi_lon_deg",
    "gpi_alt_m",
    "gpi_relative_alt_m",
    "gpi_vx_m_s",
    "gpi_vy_m_s",
    "gpi_vz_m_s",
    "gpi_hdg_deg",
    "lpn_time_boot_ms",
    "lpn_x_m",
    "lpn_y_m",
    "lpn_z_m",
    "lpn_vx_m_s",
    "lpn_vy_m_s",
    "lpn_vz_m_s",
]


def _empty_state() -> Dict[str, Optional[float]]:
    return {name: None for name in MAVLINK_COMPARE_HEADER if name not in {"msg_type"}}


def _value(value: Optional[float]) -> str:
    if value is None:
        return ""
    if isinstance(value, int):
        return str(value)
    return f"{float(value):.9f}"


class PassiveMAVLinkCompareLogger:
    """Record SIM_STATE, GLOBAL_POSITION_INT, and LOCAL_POSITION_NED from tap2 ports."""

    def __init__(
        self,
        *,
        experiment_dir: str,
        drone_ids: Iterable[int],
        port_base: int = 14751,
        port_step: int = 10,
        hz: float = 20.0,
        heartbeat_timeout_s: float = 12.0,
        subdir_name: str = "mavlink_compare",
    ) -> None:
        self.experiment_dir = os.path.abspath(str(experiment_dir))
        self.output_dir = os.path.join(self.experiment_dir, subdir_name)
        self.drone_ids = [int(did) for did in drone_ids]
        self.port_base = int(port_base)
        self.port_step = int(port_step)
        self.hz = float(hz)
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)

        self._stop_event = threading.Event()
        self._threads: List[threading.Thread] = []
        self._files: Dict[int, TextIO] = {}
        self._writers: Dict[int, csv.writer] = {}
        self._start_time: Optional[float] = None
        self._running = False

    def start(self) -> None:
        """Create output files and start one daemon receiver thread per drone."""
        if self._running:
            return
        os.makedirs(self.output_dir, exist_ok=True)
        self._stop_event.clear()
        self._start_time = time.time()
        self._running = True

        for drone_id in self.drone_ids:
            path = os.path.join(self.output_dir, f"drone_{drone_id}_mavlink_compare.csv")
            f = open(path, "w", encoding="utf-8", newline="")
            writer = csv.writer(f)
            writer.writerow(MAVLINK_COMPARE_HEADER)
            f.flush()
            self._files[drone_id] = f
            self._writers[drone_id] = writer

            port = self.port_base + (drone_id - 1) * self.port_step
            thread = threading.Thread(
                target=self._run_drone,
                args=(drone_id, port),
                daemon=True,
                name=f"mavlink-compare-drone-{drone_id}",
            )
            thread.start()
            self._threads.append(thread)

        logger.info("MAVLink compare logger writing to: %s", self.output_dir)

    def stop(self) -> None:
        """Signal all receiver threads to stop and wait briefly for shutdown."""
        self._stop_event.set()
        for thread in list(self._threads):
            thread.join(timeout=2.0)
        self._threads.clear()
        self._running = False

    def close(self) -> None:
        """Stop threads and close all CSV files."""
        self.stop()
        for drone_id, f in list(self._files.items()):
            try:
                f.close()
            except Exception as exc:
                logger.warning("Drone %s: failed closing MAVLink compare log: %s", drone_id, exc)
        self._files.clear()
        self._writers.clear()

    def _run_drone(self, drone_id: int, port: int) -> None:
        connection_string = f"udp:127.0.0.1:{port}"
        master: Any = None
        try:
            master = mavutil.mavlink_connection(connection_string)
            heartbeat = master.wait_heartbeat(timeout=self.heartbeat_timeout_s)
            if heartbeat is None:
                logger.warning(
                    "Drone %s: no heartbeat on MAVLink compare tap %s within %.1fs",
                    drone_id,
                    connection_string,
                    self.heartbeat_timeout_s,
                )
                return
            self._request_compare_streams(master)
            self._receive_loop(drone_id, master)
        except Exception as exc:
            logger.warning("Drone %s: MAVLink compare logger stopped: %s", drone_id, exc)
        finally:
            try:
                if master is not None:
                    master.close()
            except Exception:
                pass

    def _request_compare_streams(self, master: Any) -> None:
        interval_us = max(1, int(1_000_000.0 / max(0.1, self.hz)))
        for msg_id in (
            _SIM_STATE_MSG_ID,
            _GLOBAL_POSITION_INT_MSG_ID,
            _LOCAL_POSITION_NED_MSG_ID,
        ):
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
        state = _empty_state()
        receipt_count = 0
        while not self._stop_event.is_set():
            msg = master.recv_match(
                type=["SIM_STATE", "GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"],
                blocking=True,
                timeout=0.5,
            )
            if msg is None:
                continue
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                continue
            receipt_count += 1
            if msg_type == "SIM_STATE":
                self._update_sim_state(state, msg)
            elif msg_type == "GLOBAL_POSITION_INT":
                self._update_global_position_int(state, msg)
            elif msg_type == "LOCAL_POSITION_NED":
                self._update_local_position_ned(state, msg)
            self._write_row(drone_id, msg_type, receipt_count, state)

    def _write_row(
        self,
        drone_id: int,
        msg_type: str,
        receipt_count: int,
        state: Dict[str, Optional[float]],
    ) -> None:
        now = time.time()
        start_time = self._start_time if self._start_time is not None else now
        row_state = dict(state)
        row_state["timestamp_unix_s"] = now
        row_state["t_rel_s"] = now - start_time
        row_state["drone_id"] = drone_id
        row_state["msg_receipt_count"] = receipt_count
        row = []
        for column in MAVLINK_COMPARE_HEADER:
            if column == "msg_type":
                row.append(msg_type)
            else:
                row.append(_value(row_state.get(column)))
        writer = self._writers.get(drone_id)
        file_handle = self._files.get(drone_id)
        if writer is None or file_handle is None:
            return
        writer.writerow(row)
        file_handle.flush()

    @staticmethod
    def _update_sim_state(state: Dict[str, Optional[float]], msg: Any) -> None:
        lat_deg, lon_deg = sim_state_lat_lon_deg(msg)
        state.update(
            {
                "sim_time_boot_ms": _optional_float(getattr(msg, "time_boot_ms", None)),
                "sim_lat_deg": lat_deg,
                "sim_lon_deg": lon_deg,
                "sim_alt_m": _optional_float(getattr(msg, "alt", None)),
                "sim_roll_rad": _optional_float(getattr(msg, "roll", None)),
                "sim_pitch_rad": _optional_float(getattr(msg, "pitch", None)),
                "sim_yaw_rad": _optional_float(getattr(msg, "yaw", None)),
                "sim_vn_m_s": _optional_float(getattr(msg, "vn", None)),
                "sim_ve_m_s": _optional_float(getattr(msg, "ve", None)),
                "sim_vd_m_s": _optional_float(getattr(msg, "vd", None)),
                "sim_xacc": _optional_float(getattr(msg, "xacc", None)),
                "sim_yacc": _optional_float(getattr(msg, "yacc", None)),
                "sim_zacc": _optional_float(getattr(msg, "zacc", None)),
            }
        )

    @staticmethod
    def _update_global_position_int(state: Dict[str, Optional[float]], msg: Any) -> None:
        hdg = _optional_float(getattr(msg, "hdg", None))
        if hdg == 65535.0:
            hdg = None
        state.update(
            {
                "gpi_time_boot_ms": _optional_float(getattr(msg, "time_boot_ms", None)),
                "gpi_lat_deg": _optional_float(getattr(msg, "lat", None), scale=1.0e-7),
                "gpi_lon_deg": _optional_float(getattr(msg, "lon", None), scale=1.0e-7),
                "gpi_alt_m": _optional_float(getattr(msg, "alt", None), scale=0.001),
                "gpi_relative_alt_m": _optional_float(
                    getattr(msg, "relative_alt", None), scale=0.001
                ),
                "gpi_vx_m_s": _optional_float(getattr(msg, "vx", None), scale=0.01),
                "gpi_vy_m_s": _optional_float(getattr(msg, "vy", None), scale=0.01),
                "gpi_vz_m_s": _optional_float(getattr(msg, "vz", None), scale=0.01),
                "gpi_hdg_deg": hdg * 0.01 if hdg is not None else None,
            }
        )

    @staticmethod
    def _update_local_position_ned(state: Dict[str, Optional[float]], msg: Any) -> None:
        state.update(
            {
                "lpn_time_boot_ms": _optional_float(getattr(msg, "time_boot_ms", None)),
                "lpn_x_m": _optional_float(getattr(msg, "x", None)),
                "lpn_y_m": _optional_float(getattr(msg, "y", None)),
                "lpn_z_m": _optional_float(getattr(msg, "z", None)),
                "lpn_vx_m_s": _optional_float(getattr(msg, "vx", None)),
                "lpn_vy_m_s": _optional_float(getattr(msg, "vy", None)),
                "lpn_vz_m_s": _optional_float(getattr(msg, "vz", None)),
            }
        )


def _optional_float(value: Any, *, scale: float = 1.0) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value) * float(scale)
    except (TypeError, ValueError):
        return None

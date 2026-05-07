"""
MAVLink worker: single-threaded, thread-safe access to one drone connection.

All pymavlink operations (recv_msg, rc_channels_override_send, set_mode, etc.)
run in one dedicated thread. Callers use get_position(), get_attitude(),
send_rc_override() and run_init_sequence() which enqueue commands or read
from thread-safe state cache (pose from SIM_STATE, NED from lat/lon vs home).
"""

import logging
import queue
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from pymavlink import mavutil

from core.mavlink.geo_ned import (
    home_position_lat_lon_alt_m,
    ned_metres_from_home,
    sim_state_lat_lon_deg,
)

logger = logging.getLogger(__name__)

_SIM_STATE_MSG_ID: int = int(
    getattr(mavutil.mavlink, "MAVLINK_MSG_ID_SIM_STATE", 108)
)
_HOME_POSITION_MSG_ID: int = int(
    getattr(mavutil.mavlink, "MAVLINK_MSG_ID_HOME_POSITION", 242)
)

# Main thread must not call recv / send on mavutil after the worker thread starts.
_MAX_RECV_DRAIN_PER_ITER = 512
_DEFAULT_TELEMETRY_HZ = 50


class MAVLinkWorker:
    """
    Thread-safe MAVLink access for one drone.

    One dedicated thread performs all recv_msg() and mav.xxx_send() calls.
    Callers send commands via queue; state (position, attitude) is read via
    get_position() / get_attitude() under lock.
    """

    def __init__(self, connection_string: str, drone_id: int) -> None:
        """
        Initialize the worker (connection and thread start in start()).

        Args:
            connection_string: e.g. 'udp:127.0.0.1:14551'.
            drone_id: Drone identifier for logging and state.
        """
        self.connection_string = connection_string
        self.drone_id = drone_id
        self._command_queue: queue.Queue[Dict[str, Any]] = queue.Queue()
        self._state_lock = threading.Lock()
        # Condition waits for new position samples (SIM_STATE-derived NED).
        self._pos_cond = threading.Condition(self._state_lock)
        self._last_position: Optional[Dict[str, float]] = None
        self._last_attitude: Optional[Dict[str, float]] = None
        self._pos_seq: int = 0
        self._att_seq: int = 0
        self._home_lat_deg: float = 0.0
        self._home_lon_deg: float = 0.0
        self._home_alt_m: float = 0.0
        self._home_initialized: bool = False
        # time_boot from SIM_STATE pose (s); HEARTBEAT refills when SIM_STATE omits time_boot_ms.
        self._last_vehicle_sitl_time_boot_sec: Optional[float] = None
        self._last_position_sitl_time_boot_sec: Optional[float] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._master: Any = None

    def start(self) -> None:
        """Connect to the vehicle and start the single MAVLink thread."""
        if self._running:
            return
        conn_kw: Dict[str, Any] = {}
        if self.connection_string.startswith("tcp:"):
            conn_kw["retries"] = 25
        self._master = mavutil.mavlink_connection(self.connection_string, **conn_kw)
        hb = None
        try:
            hb = self._master.wait_heartbeat(timeout=getattr(self, "_heartbeat_timeout", None))
        except TypeError:
            hb = self._master.wait_heartbeat()
        if hb is None:
            raise TimeoutError(
                f"Heartbeat timeout while connecting to {self.connection_string}"
            )
        try:
            self._prime_telemetry_streams(hz=_DEFAULT_TELEMETRY_HZ)
        except Exception as exc:
            logger.warning(
                "Drone %s: telemetry stream prime failed (continuing): %s",
                self.drone_id,
                exc,
            )
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("MAVLinkWorker started for drone %s", self.drone_id)

    def start_with_heartbeat_timeout(self, heartbeat_timeout: Optional[float]) -> None:
        """
        Start worker with an optional heartbeat timeout (seconds).

        This keeps start() backward-compatible for existing scenarios while allowing
        scenarios to explicitly validate connectivity via heartbeat.
        """
        self._heartbeat_timeout = heartbeat_timeout
        self.start()

    def _prime_telemetry_streams(self, hz: int = _DEFAULT_TELEMETRY_HZ) -> None:
        """Request HOME_POSITION + SIM_STATE intervals (SITL truth pose)."""
        if self._master is None:
            return
        m = self._master
        ts, tc = m.target_system, m.target_component
        rate = max(1, min(50, int(hz)))
        interval_us = max(1, int(1e6 / rate))
        home_interval_us = max(1, int(5e5))  # 2 Hz: home for NED origin
        m.mav.command_long_send(
            ts,
            tc,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            _HOME_POSITION_MSG_ID,
            home_interval_us,
            0,
            0,
            0,
            0,
            0,
        )
        m.mav.command_long_send(
            ts,
            tc,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            _SIM_STATE_MSG_ID,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def stop(self) -> None:
        """Stop the MAVLink thread. Safe to call from any thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._master = None
        logger.info("MAVLinkWorker stopped for drone %s", self.drone_id)

    def _run_loop(self) -> None:
        """Single MAVLink thread: process queue, then read messages, update state."""
        while self._running and self._master:
            try:
                cmd = self._command_queue.get_nowait()
                self._execute_command(cmd)
            except queue.Empty:
                pass

            drained = 0
            while (
                self._running
                and self._master
                and drained < _MAX_RECV_DRAIN_PER_ITER
            ):
                msg = self._master.recv_msg()
                if msg is None:
                    break
                drained += 1
                mt = msg.get_type()
                if mt == "BAD_DATA":
                    continue
                msg_time_boot_ms = getattr(msg, "time_boot_ms", None)
                if msg_time_boot_ms is not None:
                    with self._state_lock:
                        self._last_vehicle_sitl_time_boot_sec = (
                            float(msg_time_boot_ms) / 1000.0
                        )
                if mt == "HOME_POSITION":
                    hlat, hlon, halt = home_position_lat_lon_alt_m(msg)
                    with self._state_lock:
                        self._home_lat_deg = hlat
                        self._home_lon_deg = hlon
                        self._home_alt_m = halt
                        self._home_initialized = True
                elif mt == "SIM_STATE":
                    lat, lon = sim_state_lat_lon_deg(msg)
                    alt_m = float(getattr(msg, "alt", 0.0))
                    with self._state_lock:
                        if msg_time_boot_ms is not None:
                            sitl_tb_pose: Optional[float] = (
                                float(msg_time_boot_ms) / 1000.0
                            )
                        else:
                            sitl_tb_pose = self._last_vehicle_sitl_time_boot_sec
                        if not self._home_initialized:
                            self._home_lat_deg = lat
                            self._home_lon_deg = lon
                            self._home_alt_m = alt_m
                            self._home_initialized = True
                            logger.info(
                                "Drone %s: NED origin latched from first SIM_STATE "
                                "(HOME_POSITION not yet applied)",
                                self.drone_id,
                            )
                        x, y, z = ned_metres_from_home(
                            lat,
                            lon,
                            alt_m,
                            self._home_lat_deg,
                            self._home_lon_deg,
                            self._home_alt_m,
                        )
                        self._last_position = {
                            "x": x,
                            "y": y,
                            "z": z,
                            "vx": float(getattr(msg, "vn", 0.0)),
                            "vy": float(getattr(msg, "ve", 0.0)),
                            "vz": float(getattr(msg, "vd", 0.0)),
                        }
                        self._last_attitude = {
                            "rx": float(getattr(msg, "roll", 0.0)),
                            "ry": float(getattr(msg, "pitch", 0.0)),
                            "rz": float(getattr(msg, "yaw", 0.0)),
                        }
                        self._last_position_sitl_time_boot_sec = sitl_tb_pose
                        self._pos_seq += 1
                        self._att_seq += 1
                        self._pos_cond.notify_all()

            time.sleep(0.01)

    def _execute_command(self, cmd: Dict[str, Any]) -> None:
        """Run one command (only called from _run_loop)."""
        cmd_type = cmd.get("type")
        if cmd_type == "rc_override":
            self._master.mav.rc_channels_override_send(
                self._master.target_system,
                self._master.target_component,
                cmd["chan1"],
                cmd["chan2"],
                cmd["chan3"],
                cmd["chan4"],
                0,
                0,
                0,
                0,
            )
        elif cmd_type == "set_mode":
            self._master.set_mode(cmd["mode_id"])
        elif cmd_type == "arm":
            self._master.arducopter_arm()
        elif cmd_type == "takeoff":
            # MAV_CMD_NAV_TAKEOFF: param7 = altitude (m); keep default 1 unless step sets alt_m.
            alt_m = float(cmd.get("alt_m", cmd.get("altitude_m", 1.0)))
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                alt_m,
            )
        elif cmd_type == "request_position_stream":
            interval_us = int(1e6 / max(1, int(cmd.get("hz", 50))))
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                _SIM_STATE_MSG_ID,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
        elif cmd_type == "request_attitude_stream":
            interval_us = int(1e6 / max(1, int(cmd.get("hz", 50))))
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                _SIM_STATE_MSG_ID,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
        elif cmd_type == "sleep":
            time.sleep(cmd.get("sec", 0))
        elif cmd_type == "init_done":
            evt = cmd.get("event")
            if evt is not None:
                evt.set()

    def send_rc_override(
        self,
        chan1: int,
        chan2: int,
        chan3: int,
        chan4: int,
        controller: Optional[Any] = None,
    ) -> None:
        """
        Send RC_OVERRIDE (roll, pitch, throttle, yaw). Thread-safe via queue.

        Args:
            chan1: Roll.
            chan2: Pitch.
            chan3: Throttle.
            chan4: Yaw.
            controller: Optional; if has last_rc_channels and rc_channels_lock, update them.
        """
        try:
            self._command_queue.put_nowait(
                {
                    "type": "rc_override",
                    "chan1": chan1,
                    "chan2": chan2,
                    "chan3": chan3,
                    "chan4": chan4,
                }
            )
        except queue.Full:
            logger.warning("MAVLinkWorker command queue full, dropping rc_override")
        if controller is not None and hasattr(controller, "rc_channels_lock"):
            with controller.rc_channels_lock:
                controller.last_rc_channels["roll"] = chan1
                controller.last_rc_channels["pitch"] = chan2
                controller.last_rc_channels["throttle"] = chan3
                controller.last_rc_channels["yaw"] = chan4

    def send_set_mode(self, mode_id: int) -> None:
        """Enqueue set_mode. Thread-safe."""
        try:
            self._command_queue.put_nowait({"type": "set_mode", "mode_id": mode_id})
        except queue.Full:
            logger.warning("MAVLinkWorker command queue full, dropping set_mode")

    def get_position(self) -> Optional[Dict[str, float]]:
        """
        Return last SIM_STATE-derived NED pose: x,y,z from home and vn/ve/vd as vx,vy,vz.

        Thread-safe.

        Returns:
            Dict or None if no SIM_STATE received yet.
        """
        with self._state_lock:
            if self._last_position is None:
                return None
            return dict(self._last_position)

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """
        Return last roll/pitch/yaw from SIM_STATE (radians as rx, ry, rz). Thread-safe.

        Returns:
            Dict or None if no SIM_STATE received yet.
        """
        with self._state_lock:
            if self._last_attitude is None:
                return None
            return dict(self._last_attitude)

    def get_position_seq(self) -> int:
        """Return current position sequence number (one increment per SIM_STATE)."""
        with self._state_lock:
            return int(self._pos_seq)

    def wait_for_new_position(
        self, last_seq: int, timeout_s: float = 0.25
    ) -> Optional[Tuple[int, Dict[str, float], Optional[float]]]:
        """
        Block until a new SIM_STATE-derived position arrives (seq changes).

        Args:
            last_seq: Previously seen sequence number.
            timeout_s: Max seconds to wait.

        Returns:
            (new_seq, position_dict, sitl_time_boot_s) or None on timeout / if no position yet.
            ``sitl_time_boot_s`` matches the SIM_STATE sample that bumped seq (may be None).
        """
        deadline = time.time() + float(timeout_s)
        with self._pos_cond:
            while self._pos_seq <= int(last_seq) and self._running:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._pos_cond.wait(timeout=remaining)
            if self._pos_seq <= int(last_seq) or self._last_position is None:
                return None
            sitl_s = self._last_position_sitl_time_boot_sec
            return (int(self._pos_seq), dict(self._last_position), sitl_s)

    def get_attitude_seq(self) -> int:
        """Return current attitude sequence number (one increment per SIM_STATE)."""
        with self._state_lock:
            return int(self._att_seq)

    def run_init_sequence(
        self, steps: List[Dict[str, Any]], timeout: float = 60.0
    ) -> bool:
        """
        Run a list of init commands in the MAVLink thread and block until done.

        Steps are dicts with "type": "set_mode"|"arm"|"takeoff"|"request_position_stream"|
        "request_attitude_stream"|"sleep" and corresponding args. A final "init_done"
        step is appended internally; the worker sets the event so this method unblocks.

        Args:
            steps: List of command dicts (e.g. {"type": "set_mode", "mode_id": 4}).
            timeout: Max seconds to wait for init_done.

        Returns:
            True if init_done was observed within timeout, False otherwise.
        """
        evt = threading.Event()
        for s in steps:
            try:
                self._command_queue.put_nowait(dict(s))
            except queue.Full:
                logger.warning("MAVLinkWorker init queue full")
                return False
        try:
            self._command_queue.put_nowait({"type": "init_done", "event": evt})
        except queue.Full:
            return False
        return evt.wait(timeout=timeout)

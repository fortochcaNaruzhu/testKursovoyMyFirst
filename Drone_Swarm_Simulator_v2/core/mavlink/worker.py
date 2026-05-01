"""
MAVLink worker: single-threaded, thread-safe access to one drone connection.

All pymavlink operations (recv_match, rc_channels_override_send, set_mode, etc.)
run in one dedicated thread. Callers use get_position(), get_attitude(),
send_rc_override() and run_init_sequence() which enqueue commands or read
from thread-safe state cache.
"""

import logging
import queue
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from pymavlink import mavutil

logger = logging.getLogger(__name__)


class MAVLinkWorker:
    """
    Thread-safe MAVLink access for one drone.

    One dedicated thread performs all recv_match() and mav.xxx_send() calls.
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
        # Condition is used to wait for new position samples (LOCAL_POSITION_NED).
        # It shares the same underlying lock to keep state + counters consistent.
        self._pos_cond = threading.Condition(self._state_lock)
        self._last_position: Optional[Dict[str, float]] = None
        self._last_attitude: Optional[Dict[str, float]] = None
        self._pos_seq: int = 0
        self._att_seq: int = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._master: Any = None

    def start(self) -> None:
        """Connect to the vehicle and start the single MAVLink thread."""
        if self._running:
            return
        self._master = mavutil.mavlink_connection(self.connection_string)
        hb = None
        try:
            hb = self._master.wait_heartbeat(timeout=getattr(self, "_heartbeat_timeout", None))
        except TypeError:
            # Some pymavlink builds don't support a timeout kwarg.
            hb = self._master.wait_heartbeat()
        if hb is None:
            raise TimeoutError(
                f"Heartbeat timeout while connecting to {self.connection_string}"
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
            # Process one command from queue (non-blocking)
            try:
                cmd = self._command_queue.get_nowait()
                self._execute_command(cmd)
            except queue.Empty:
                pass

            # Read incoming messages (non-blocking)
            msg = self._master.recv_match(
                type=["LOCAL_POSITION_NED", "ATTITUDE"],
                blocking=False,
                timeout=0.01,
            )
            if msg is not None and msg.get_type() != "BAD_DATA":
                with self._state_lock:
                    if msg.get_type() == "LOCAL_POSITION_NED":
                        self._last_position = {
                            "x": msg.x,
                            "y": msg.y,
                            "z": msg.z,
                            "vx": getattr(msg, "vx", 0.0),
                            "vy": getattr(msg, "vy", 0.0),
                            "vz": getattr(msg, "vz", 0.0),
                        }
                        self._pos_seq += 1
                        # Wake up any logger waiting for a new sample.
                        self._pos_cond.notify_all()
                    elif msg.get_type() == "ATTITUDE":
                        self._last_attitude = {
                            "rx": msg.roll,
                            "ry": msg.pitch,
                            "rz": msg.yaw,
                        }
                        self._att_seq += 1

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
                1,
            )
        elif cmd_type == "request_position_stream":
            interval_us = int(1e6 / cmd.get("hz", 50))
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                interval_us,
                0,
                0,
                0,
                0,
                0,
            )
        elif cmd_type == "request_attitude_stream":
            interval_us = int(1e6 / cmd.get("hz", 50))
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
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
        Return last LOCAL_POSITION_NED (x, y, z, vx, vy, vz). Thread-safe.

        Returns:
            Dict or None if no position received yet.
        """
        with self._state_lock:
            if self._last_position is None:
                return None
            return dict(self._last_position)

    def get_attitude(self) -> Optional[Dict[str, float]]:
        """
        Return last ATTITUDE (roll, pitch, yaw in radians as rx, ry, rz). Thread-safe.

        Returns:
            Dict or None if no attitude received yet.
        """
        with self._state_lock:
            if self._last_attitude is None:
                return None
            return dict(self._last_attitude)

    def get_position_seq(self) -> int:
        """Return current LOCAL_POSITION_NED sequence number (monotonic)."""
        with self._state_lock:
            return int(self._pos_seq)

    def wait_for_new_position(
        self, last_seq: int, timeout_s: float = 0.25
    ) -> Optional[Tuple[int, Dict[str, float]]]:
        """
        Block until a new LOCAL_POSITION_NED sample arrives (seq changes).

        Args:
            last_seq: Previously seen sequence number.
            timeout_s: Max seconds to wait.

        Returns:
            (new_seq, position_dict) or None on timeout / if no position yet.
        """
        deadline = time.time() + float(timeout_s)
        with self._state_lock:
            while self._pos_seq <= int(last_seq) and self._running:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self._pos_cond.wait(timeout=remaining)
            if self._pos_seq <= int(last_seq) or self._last_position is None:
                return None
            return (int(self._pos_seq), dict(self._last_position))

    def get_attitude_seq(self) -> int:
        """Return current ATTITUDE sequence number (monotonic)."""
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

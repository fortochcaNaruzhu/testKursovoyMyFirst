"""
Square formation: drones fly a square pattern with PID braking.
Imports PIDRegulator, VelocityMonitor, send_rc_override from core.

Accepts --drones, --duration, --experiment-dir from launcher for batch compatibility;
currently uses fixed config (e.g. 5 drones). leader_forward_back is the main scenario
with full support of these arguments.
"""
import os
import sys

# Ensure project root is on path when run as script
_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

import argparse
import logging
import threading
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

from core.control import DroneController, PIDRegulator
from core.mavlink.utils import RC_NEUTRAL

try:
    from visualizer.position_publisher import publish_positions as _publish_positions
except ImportError:
    _publish_positions = None

# Square step commands (roll, pitch, throttle, yaw)
SQUARE_STEPS = [
    (RC_NEUTRAL, 1700, RC_NEUTRAL, RC_NEUTRAL),  # Back
    (1700, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL),  # Right
    (RC_NEUTRAL, 1300, RC_NEUTRAL, RC_NEUTRAL),  # Forward
    (1300, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL),  # Left
]
STEP_DURATION = 2  # seconds per segment

SQUARE_INIT_STEPS = [
    {"type": "set_mode", "mode_id": 4},
    {"type": "sleep", "sec": 0.5},
    {"type": "arm"},
    {"type": "sleep", "sec": 2},
    {"type": "takeoff"},
    {"type": "sleep", "sec": 5},
    {"type": "set_mode", "mode_id": 2},
    {
        "type": "rc_override",
        "chan1": RC_NEUTRAL,
        "chan2": RC_NEUTRAL,
        "chan3": RC_NEUTRAL,
        "chan4": RC_NEUTRAL,
    },
    {"type": "sleep", "sec": 0.5},
]


class SquareDroneController(DroneController):
    """DroneController with velocity PIDs and move_with_pid_braking for square pattern."""

    def __init__(
        self, config: Dict[str, Any], logging_enabled: bool = False
    ) -> None:
        """Initialize square-formation controller with velocity PIDs.

        Args:
            config: Dict with 'id', 'udp_port', 'role'.
            logging_enabled: If True, write logs to log_dir.
        """
        super().__init__(
            config,
            logging_enabled=logging_enabled,
            log_dir="logs_SQUARE_two_drones",
        )
        self.roll_velocity_pid = PIDRegulator(
            kp=100.0,
            ki=100.0,
            kd=0.0,
            integral_limit=200.0,
            output_limit=500.0,
        )
        self.pitch_velocity_pid = PIDRegulator(
            kp=100.0,
            ki=0.0,
            kd=0.0,
            integral_limit=200.0,
            output_limit=500.0,
        )
        self.last_movement_mode: Optional[str] = None

    def move_with_pid_braking(
        self,
        direction: Tuple[int, int, int, int],
        kpx: float = 2000,
        kpy: float = 2000,
        move_duration: float = 5,
        target_velocity: float = 0.0,
        max_brake_time: float = 10.0,
        velocity_threshold: float = 0.8,
        use_pid: bool = True,
    ) -> None:
        """Apply direction then brake to target_velocity with PID.

        Args:
            direction: (roll, pitch, throttle, yaw) RC channel values for move phase.
            kpx: Pitch braking gain when use_pid is False.
            kpy: Roll braking gain when use_pid is False.
            move_duration: Seconds to apply direction before braking.
            target_velocity: Target vx, vy for brake phase (m/s).
            max_brake_time: Max seconds to run brake phase.
            velocity_threshold: End brake when |vx|,|vy| below this (m/s).
            use_pid: If True, use velocity PIDs for braking; else use kpx/kpy gains.

        Returns:
            None. Sends RC_OVERRIDE via worker until brake complete or timeout.
        """
        roll, pitch, throttle, yaw = direction
        if self.last_movement_mode != "moving":
            self.roll_velocity_pid.reset()
            self.pitch_velocity_pid.reset()
        self.last_movement_mode = "moving"
        start = time.time()
        while time.time() - start < move_duration:
            if self.worker:
                self.worker.send_rc_override(
                    roll, pitch, throttle, yaw, controller=self
                )
            time.sleep(0.1)
        self.last_movement_mode = "braking"
        start_time = time.time()
        last_update_time = time.time()
        while time.time() - start_time < max_brake_time:
            current_time = time.time()
            dt = current_time - last_update_time
            if dt <= 0:
                dt = 0.05
            last_update_time = current_time
            if self.velocity_monitor is None:
                break
            current_velocity = self.velocity_monitor.get_velocity()
            current_x_velocity = current_velocity["vx"]
            current_y_velocity = current_velocity["vy"]
            if (
                abs(current_x_velocity) < velocity_threshold
                and abs(current_y_velocity) < velocity_threshold
            ):
                self.roll_velocity_pid.reset()
                self.pitch_velocity_pid.reset()
                break
            error_vx = target_velocity - current_x_velocity
            error_vy = target_velocity - current_y_velocity
            if use_pid:
                brake_output_pitch = self.pitch_velocity_pid.update(
                    error_vx, dt
                )
                brake_intensity_x = int(abs(brake_output_pitch))
                brake_intensity_x = min(500, max(50, brake_intensity_x))
                brake_direction_x = -1 if error_vx > 0 else 1
                brake_pitch = RC_NEUTRAL + (
                    brake_direction_x * brake_intensity_x
                )
                brake_output_roll = self.roll_velocity_pid.update(error_vy, dt)
                brake_intensity_y = int(abs(brake_output_roll))
                brake_intensity_y = min(500, max(50, brake_intensity_y))
                brake_direction_y = 1 if error_vy > 0 else -1
                brake_roll = RC_NEUTRAL + (
                    brake_direction_y * brake_intensity_y
                )
            else:
                if kpx > 0:
                    brake_intensity_x = int(abs(error_vx) * kpx)
                    brake_intensity_x = min(500, max(50, brake_intensity_x))
                    brake_direction_x = 1 if error_vx > 0 else -1
                    brake_pitch = RC_NEUTRAL + (
                        brake_direction_x * brake_intensity_x
                    )
                else:
                    brake_pitch = RC_NEUTRAL
                if kpy > 0:
                    brake_intensity_y = int(abs(error_vy) * kpy)
                    brake_intensity_y = min(500, max(50, brake_intensity_y))
                    brake_direction_y = -1 if error_vy > 0 else 1
                    brake_roll = RC_NEUTRAL + (
                        brake_direction_y * brake_intensity_y
                    )
                else:
                    brake_roll = RC_NEUTRAL
            if self.worker:
                self.worker.send_rc_override(
                    brake_roll, brake_pitch, RC_NEUTRAL, RC_NEUTRAL,
                    controller=self,
                )
            time.sleep(0.05)


def square_pattern(
    controller: SquareDroneController,
    step_duration: float = 2,
    kpx: float = 100,
    kpy: float = 100,
    use_pid: bool = True,
    velocity_threshold: float = 0.2,
) -> None:
    """Run square pattern with PID braking (back, right, forward, left) in a loop.

    Args:
        controller: Drone controller (connected and initialized).
        step_duration: Seconds per segment.
        kpx: Pitch braking gain.
        kpy: Roll braking gain.
        use_pid: Use velocity PID for braking.
        velocity_threshold: Velocity threshold to end brake phase.
    """
    while True:
        for step in SQUARE_STEPS:
            controller.move_with_pid_braking(
                step,
                kpx=kpx,
                kpy=kpy,
                move_duration=step_duration,
                target_velocity=0.0,
                max_brake_time=5.0,
                velocity_threshold=velocity_threshold,
                use_pid=use_pid,
            )
            if controller.worker:
                controller.worker.send_rc_override(
                    RC_NEUTRAL,
                    RC_NEUTRAL,
                    RC_NEUTRAL,
                    RC_NEUTRAL,
                    controller=controller,
                )
            time.sleep(1)


def initialize_drone_parallel(
    controller: SquareDroneController, init_barrier: threading.Barrier
) -> None:
    """Connect, initialize, start RC keepalive, then wait on barrier (called from worker thread)."""
    try:
        controller.connect()
        controller.initialize(SQUARE_INIT_STEPS)
        controller.start_rc_keepalive()
        init_barrier.wait()
    except Exception:
        logger.exception("Drone init failed")
        raise


def main() -> None:
    """Parse args, create controllers, run square pattern.

    Accepts same sync parameters as launcher: --exchange-hz, --control-hz, --log-hz
    for coordinate exchange rate, control loop rate, and CSV/log write rate.
    """
    parser = argparse.ArgumentParser(description="Square formation (square_pid)")
    parser.add_argument("--drones", type=int, default=5, help="Number of drones (launcher/batch)")
    parser.add_argument("--duration", type=float, default=0, help="Experiment duration (s); 0 = no limit")
    parser.add_argument("--experiment-dir", type=str, default=None, help="Experiment log folder")
    parser.add_argument(
        "--control-hz",
        type=float,
        default=50.0,
        help="Control loop rate (Hz); 0 = no limit (default 50)",
    )
    parser.add_argument(
        "--log-hz",
        type=float,
        default=0.0,
        help="Max log/CSV write rate (Hz); 0 = every exchange step (default 0)",
    )
    parser.add_argument(
        "--exchange-hz",
        type=float,
        default=50.0,
        help="Coordinate exchange / sync loop rate (Hz); match SITL (default 50)",
    )
    args = parser.parse_args()

    exchange_hz = max(0.0, float(args.exchange_hz))
    loop_period_sec = (1.0 / exchange_hz) if exchange_hz > 0 else 0.02  # default 50 Hz
    log_hz = max(0.0, float(args.log_hz))

    num_drones = max(1, args.drones)
    DRONES_CONFIG = [
        {"id": i + 1, "udp_port": 14551 + i * 10, "role": "square"}
        for i in range(num_drones)
    ]
    controllers = []
    for config in DRONES_CONFIG:
        controllers.append(SquareDroneController(config, logging_enabled=True))
    init_barrier = threading.Barrier(len(controllers) + 1)
    for controller in controllers:
        threading.Thread(
            target=initialize_drone_parallel,
            args=(controller, init_barrier),
            daemon=False,
        ).start()
    try:
        init_barrier.wait(timeout=30)
    except threading.BrokenBarrierError:
        return
    time.sleep(2)
    try:
        square_threads = []
        for controller in controllers:
            t = threading.Thread(
                target=square_pattern,
                args=(controller, STEP_DURATION, 100, 100, True),
                kwargs={"velocity_threshold": 0.2},
                daemon=True,
            )
            t.start()
            square_threads.append(t)
        last_log_time = 0.0
        rates_shared: Dict[str, Optional[float]] = {"exchange_hz": exchange_hz if exchange_hz > 0 else None}
        while True:
            t0 = time.time()
            # Publish positions for 2D visualizer (same NED convention as leader_forward_back)
            positions: Dict[int, Dict[str, float]] = {}
            for c in controllers:
                try:
                    pos = c.get_my_position()
                    if pos:
                        did = c.config["id"]
                        positions[did] = {
                            **pos,
                            "y": pos.get("y", 0) + (did - 1) * 2.0,
                        }
                except Exception:
                    pass
            if _publish_positions is not None and positions:
                try:
                    _publish_positions(positions, rates=rates_shared)
                except Exception:
                    pass
            do_log = log_hz <= 0 or (log_hz > 0 and (t0 - last_log_time) >= (1.0 / log_hz))
            if do_log:
                for c in controllers:
                    if c.logging_enabled and hasattr(c, "logfile") and c.velocity_monitor:
                        try:
                            vel = c.velocity_monitor.get_velocity()
                            now = datetime.now()
                            ts = f"{now.second:02d}.{now.microsecond // 1000:03d}"
                            c.logfile.write(f"'t': {ts}, {vel}\n")
                        except Exception:
                            pass
                if log_hz > 0:
                    last_log_time = t0
            elapsed = time.time() - t0
            sleep_sec = loop_period_sec - elapsed
            if sleep_sec > 0:
                time.sleep(sleep_sec)
    except KeyboardInterrupt:
        for controller in controllers:
            controller.stop()


if __name__ == "__main__":
    main()

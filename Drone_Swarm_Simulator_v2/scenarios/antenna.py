"""
Сценарий «антенна» (antenna):
4 дрона, HEARTBEAT-check, взлёт и построение «вертикальной антенны».

- Дрон 1 — якорь (anchor): удерживает (x,y,z) на месте.
- Дроны 2..4 — агенты: удерживают (x,y) якоря и высоту над ним
  с равным шагом по вертикали.

Управление только через RC override: roll/pitch/yaw нейтраль, throttle — PID по ошибке высоты.
"""

from __future__ import annotations

import logging
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from core.control import DroneController, PIDRegulator
from core.mavlink.utils import RC_NEUTRAL

try:
    from visualizer.position_publisher import publish_positions as _publish_positions
except ImportError:
    _publish_positions = None

logger = logging.getLogger(__name__)

START_TIME: float = 0.0

TAKEOFF_ALT_M = 1.0
DEFAULT_SPACING_M = 0.8

CONTROL_HZ = 20.0
CONTROL_DT = 1.0 / CONTROL_HZ

# ПИД-ограничения для RC (в PWM-дельтах от 1500)
# Снижены, чтобы убрать раскачку и «кружение» вокруг точки.
XY_OUTPUT_LIMIT = 120.0
Z_OUTPUT_LIMIT = 160.0

XY_DEADBAND_M = 0.05
Z_DEADBAND_M = 0.06

# В SITL каждый дрон имеет home с east-offset (y), поэтому для общей NED рамки:
# y_common = y_local + (id-1)*2.0 (как в leader_forward_back / linear_chain2)
_HOME_Y_OFFSET_STEP_M = 2.0

INIT_STEPS = [
    {"type": "set_mode", "mode_id": 4},  # GUIDED
    {"type": "sleep", "sec": 0.5},
    {"type": "arm"},
    {"type": "sleep", "sec": 2.0},
    {"type": "takeoff"},  # worker uses ~1m takeoff in command_long
    {"type": "sleep", "sec": 3.0},
    {"type": "set_mode", "mode_id": 16},  # POSHOLD
    {
        "type": "rc_override",
        "chan1": RC_NEUTRAL,
        "chan2": RC_NEUTRAL,
        "chan3": RC_NEUTRAL,
        "chan4": RC_NEUTRAL,
    },
    {"type": "sleep", "sec": 0.3},
    {"type": "request_position_stream", "hz": 30},
    {"type": "sleep", "sec": 0.2},
]


def _did_offset_y(did: int) -> float:
    return float(did - 1) * _HOME_Y_OFFSET_STEP_M


def _pos_common(controller: DroneController) -> Dict[str, float]:
    """Position in a common NED frame (align per-drone homes)."""
    raw = controller.get_my_position()
    did = int(controller.config["id"])
    return {**raw, "y": float(raw.get("y", 0.0)) + _did_offset_y(did)}


def _clamp_rc(pwm: int, lo: int = 1100, hi: int = 1900) -> int:
    return max(lo, min(hi, int(pwm)))


def _deadband(x: float, band: float) -> float:
    return 0.0 if abs(x) < band else x


def _wait_for_takeoff(
    controller: DroneController,
    target_alt_m: float,
    timeout_s: float = 20.0,
    tol_m: float = 0.30,
) -> bool:
    """Wait until NED z is close to -target_alt_m."""
    target_z = -float(target_alt_m)
    t0 = time.time()
    last_pos: Optional[Dict[str, float]] = None
    while time.time() - t0 < timeout_s:
        pos = controller.get_position()
        if pos is not None:
            last_pos = pos
            z = float(pos.get("z", 0.0))
            if abs(z - target_z) <= tol_m:
                return True
        time.sleep(0.1)
    logger.warning(
        "[antenna] Drone %s: takeoff not confirmed within timeout. Last pos: %s",
        controller.config.get("id"),
        last_pos,
    )
    return False


def initialize_drone_antenna(
    controller: DroneController,
    init_barrier: threading.Barrier,
    *,
    heartbeat_timeout_s: Optional[float],
    position_timeout_s: float = 10.0,
    barrier_timeout_sec: float = 60.0,
) -> None:
    """Heartbeat -> initialize -> keepalive -> confirm position stream -> barrier."""
    try:
        did = controller.config.get("id")
        if heartbeat_timeout_s is not None:
            logger.info(
                "[antenna] Drone %s: connecting with heartbeat timeout %.1fs",
                did,
                float(heartbeat_timeout_s),
            )
            controller.connect_with_heartbeat_timeout(float(heartbeat_timeout_s))
        else:
            controller.connect()

        controller.initialize(list(INIT_STEPS))
        controller.start_rc_keepalive()

        t0 = time.time()
        while time.time() - t0 < float(position_timeout_s):
            if controller.get_position() is not None:
                break
            time.sleep(0.1)
        else:
            raise TimeoutError("No LOCAL_POSITION_NED after initialization")

        init_barrier.wait(timeout=float(barrier_timeout_sec))
    except Exception:
        try:
            init_barrier.abort()
        except Exception:
            pass
        logger.exception("[antenna] Drone init failed (id=%s)", controller.config.get("id"))


def _pid_xy() -> Tuple[PIDRegulator, PIDRegulator]:
    # Мягче по P и больше демпфирование по D + фильтр производной.
    roll_pid = PIDRegulator(
        kp=170.0,
        ki=0.0,
        kd=140.0,
        integral_limit=60.0,
        output_limit=XY_OUTPUT_LIMIT,
        derivative_alpha=0.65,
    )
    pitch_pid = PIDRegulator(
        kp=220.0,
        ki=0.0,
        kd=180.0,
        integral_limit=60.0,
        output_limit=XY_OUTPUT_LIMIT,
        derivative_alpha=0.65,
    )
    return roll_pid, pitch_pid


def _pid_z(output_limit: float) -> PIDRegulator:
    return PIDRegulator(
        kp=260.0,
        ki=0.0,
        kd=220.0,
        integral_limit=80.0,
        output_limit=float(output_limit),
        derivative_alpha=0.7,
    )


def _antenna_control_loop(
    controller: DroneController,
    *,
    anchor_id: int,
    spacing_m: float,
    duration_s: float,
) -> None:
    """Hold x,y at anchor and z at anchor - k*spacing (NED)."""
    global START_TIME
    did = int(controller.config["id"])
    if controller.worker is None:
        return

    # Anchor keeps its own x,y; others follow anchor.
    roll_pid, pitch_pid = _pid_xy()
    z_pid = _pid_z(output_limit=Z_OUTPUT_LIMIT)

    # Target altitude in meters (up is positive), then to NED z (up is negative).
    target_alt_m = float(TAKEOFF_ALT_M + max(0, did - anchor_id) * spacing_m)
    target_z = -target_alt_m

    while True:
        if duration_s > 0 and (time.time() - START_TIME) >= duration_s:
            return

        my = _pos_common(controller)
        if did == anchor_id:
            # Anchor: hold current x,y (no drift correction) and hold altitude at TAKEOFF_ALT_M.
            target_x = float(my.get("x", 0.0))
            target_y = float(my.get("y", 0.0))
            target_z = -float(TAKEOFF_ALT_M)
        else:
            anchor_pos = controller.get_other_drones_positions().get(int(anchor_id))
            if anchor_pos is None:
                controller.worker.send_rc_override(
                    RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, controller=controller
                )
                time.sleep(CONTROL_DT)
                continue
            target_x = float(anchor_pos.get("x", 0.0))
            target_y = float(anchor_pos.get("y", 0.0))

        err_x = _deadband(float(target_x) - float(my.get("x", 0.0)), XY_DEADBAND_M)
        err_y = _deadband(float(target_y) - float(my.get("y", 0.0)), XY_DEADBAND_M)
        err_z = _deadband(float(target_z) - float(my.get("z", 0.0)), Z_DEADBAND_M)

        # XY -> pitch/roll, Z -> throttle. In NED, negative z is up.
        pitch_out = pitch_pid.update(err_x, dt=CONTROL_DT)
        roll_out = roll_pid.update(err_y, dt=CONTROL_DT)
        thr_out = z_pid.update(err_z, dt=CONTROL_DT)

        pitch = _clamp_rc(RC_NEUTRAL - int(pitch_out))
        roll = _clamp_rc(RC_NEUTRAL + int(roll_out))
        throttle = _clamp_rc(RC_NEUTRAL - int(thr_out))
        yaw = RC_NEUTRAL

        controller.worker.send_rc_override(roll, pitch, throttle, yaw, controller=controller)
        time.sleep(CONTROL_DT)


def _stop_all(controllers: List[DroneController]) -> None:
    for c in controllers:
        try:
            c.stop()
        except Exception:
            pass


def main() -> None:
    import argparse

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    parser = argparse.ArgumentParser(description="Antenna: vertical stack above anchor (rc_override)")
    parser.add_argument("--drones", type=int, default=4, help="Launcher compatibility; coerced to 4")
    parser.add_argument("--duration", type=float, default=0.0, help="Run duration (s); 0 = infinite")
    parser.add_argument("--heartbeat-timeout", type=float, default=12.0, help="Heartbeat timeout (s)")
    parser.add_argument("--exchange-hz", type=float, default=50.0, help="Accepted for launcher compatibility; ignored.")
    parser.add_argument("--spacing", type=float, default=DEFAULT_SPACING_M, help="Vertical spacing between drones (m)")
    parser.add_argument("--anchor-id", type=int, default=1, help="Anchor drone id (default 1)")
    args = parser.parse_args()

    num_drones = int(args.drones)
    if num_drones != 4:
        logger.info("[antenna] Coercing --drones from %d to 4.", num_drones)
        num_drones = 4

    if args.exchange_hz != 0:
        logger.info("[antenna] Ignoring --exchange-hz=%.1f (no exchange loop).", float(args.exchange_hz))

    anchor_id = int(args.anchor_id)
    if anchor_id < 1 or anchor_id > num_drones:
        anchor_id = 1

    drones_config: List[Dict[str, object]] = [
        {"id": i + 1, "udp_port": 14551 + i * 10, "role": "antenna"} for i in range(num_drones)
    ]
    controllers: List[DroneController] = [DroneController(cfg, logging_enabled=False) for cfg in drones_config]

    init_barrier = threading.Barrier(len(controllers) + 1)
    for c in controllers:
        threading.Thread(
            target=initialize_drone_antenna,
            args=(c, init_barrier),
            kwargs={"heartbeat_timeout_s": float(args.heartbeat_timeout), "position_timeout_s": 10.0},
            daemon=False,
        ).start()

    try:
        init_barrier.wait(timeout=75.0)
    except threading.BrokenBarrierError:
        logger.error("[antenna] Init barrier broken; stopping.")
        _stop_all(controllers)
        return

    time.sleep(2.0)

    # Exchange positions (simple snapshot broadcast) so followers can see anchor in common frame.
    def exchange_positions_loop() -> None:
        last_pub = 0.0
        pub_period = 1.0 / 20.0  # visualizer update rate (Hz)
        while True:
            pos_common: Dict[int, Dict[str, float]] = {}
            for c in controllers:
                did = int(c.config["id"])
                pos_common[did] = _pos_common(c)
            for c in controllers:
                my_id = int(c.config["id"])
                for did, pos in pos_common.items():
                    if did != my_id:
                        c.update_other_drone_position(did, pos)

            now = time.time()
            if _publish_positions is not None and (now - last_pub) >= pub_period:
                try:
                    _publish_positions(pos_common, rates={"exchange_hz": 1.0 / 0.02})
                except Exception:
                    pass
                last_pub = now
            time.sleep(0.02)

    threading.Thread(target=exchange_positions_loop, daemon=True).start()

    global START_TIME
    START_TIME = time.time()

    # Confirm takeoff for all (non-fatal warnings).
    for c in controllers:
        _wait_for_takeoff(c, target_alt_m=TAKEOFF_ALT_M, timeout_s=25.0, tol_m=0.35)

    duration_s = float(args.duration)
    spacing_m = max(0.2, float(args.spacing))

    control_threads: List[threading.Thread] = []
    for c in controllers:
        t = threading.Thread(
            target=_antenna_control_loop,
            args=(c,),
            kwargs={
                "anchor_id": anchor_id,
                "spacing_m": spacing_m,
                "duration_s": duration_s,
            },
            daemon=False,
        )
        t.start()
        control_threads.append(t)

    try:
        if duration_s > 0:
            for t in control_threads:
                t.join()
        else:
            threading.Event().wait()
    except KeyboardInterrupt:
        logger.info("[antenna] KeyboardInterrupt: stopping.")
    finally:
        _stop_all(controllers)


if __name__ == "__main__":
    main()

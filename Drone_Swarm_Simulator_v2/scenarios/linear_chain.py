"""
Линейная цепь (linear_chain): drones form an evenly spaced chain on a segment.

Behavior:
- All drones take off and switch to POS_HOLD.
- Endpoints are drones with min and max id; they move to fixed targets A and B (common frame).
- Internal drones are decentralized: each drone observes other drones' coordinates, finds its
  immediate neighbors along the segment (by projection), and moves toward the midpoint between
  those neighbors, converging to equal spacing without a central orchestrator.

Important coordinate convention:
SITL homes are offset East by (did-1)*2m. We exchange positions in a *common* NED frame by adding
(did-1)*2.0 to local y. When commanding a drone toward a common-frame target, we must convert
target y back to the drone's local frame by subtracting that offset.
"""

import logging
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

# Ensure project root is on path when run as script (e.g. python scenarios/linear_chain.py)
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

START_TIME = 0.0

TAKEOFF_ALT_M = 1.0
SEGMENT_LENGTH_M = 8.0
ENDPOINT_REACHED_THRESH_M = 0.5
INTERNAL_REACHED_THRESH_M = 0.35

# RC/PID tuning (meters -> PWM): match the working scale used in leader_forward_back
# follower control (kp ~ 500 with output_limit 200).
PID_KP = 500.0
PID_KI = 0.0
PID_KD = 0.0
PID_OUTPUT_LIMIT = 200.0
PID_INTEGRAL_LIMIT = 100.0

# Control loop rate for per-drone chain behavior.
CONTROL_HZ = 20.0
ERROR_DEADBAND_M = 0.12

# Endpoint tuning: move along X confidently, correct Y gently (prevents orbiting).
ENDPOINT_X_KP = 500.0
ENDPOINT_Y_KP = 180.0
ENDPOINT_OUTPUT_LIMIT = 200.0

# Internal drones should be gentler to avoid overshoot + order swaps.
INTERNAL_PID_KP = 220.0
INTERNAL_PID_KI = 0.0
INTERNAL_PID_KD = 120.0
INTERNAL_PID_OUTPUT_LIMIT = 120.0
INTERNAL_PID_DERIVATIVE_ALPHA = 0.85  # stronger low-pass to reduce noise-driven wobble
TARGET_SMOOTHING_ALPHA = 0.90  # 0..1, higher = smoother (slower)
NEIGHBOR_HYSTERESIS_M = 0.6  # keep neighbors unless swap is "clear" by this margin

# Extra damping for internal drones along the segment direction (projection space).
# This targets the slow non-decaying oscillation by penalizing velocity along the segment.
INTERNAL_S_KP_PWM_PER_M = 140.0
INTERNAL_S_KV_PWM_PER_MPS = 110.0
INTERNAL_S_OUTPUT_LIMIT_PWM = 140.0
INTERNAL_S_DEADBAND_M = 0.18

INIT_STEPS = [
    {"type": "set_mode", "mode_id": 4},  # GUIDED
    {"type": "sleep", "sec": 0.5},
    {"type": "arm"},
    {"type": "sleep", "sec": 2},
    {"type": "takeoff"},
    {"type": "sleep", "sec": 5},
    {"type": "set_mode", "mode_id": 16},  # POS_HOLD
    {
        "type": "rc_override",
        "chan1": RC_NEUTRAL,
        "chan2": RC_NEUTRAL,
        "chan3": RC_NEUTRAL,
        "chan4": RC_NEUTRAL,
    },
    {"type": "sleep", "sec": 0.5},
    {"type": "request_position_stream", "hz": 50},
    {"type": "request_attitude_stream", "hz": 50},
    {"type": "sleep", "sec": 0.2},
]


def _did_offset_y(did: int) -> float:
    return (did - 1) * 2.0


def _my_position_common(controller: DroneController) -> Dict[str, float]:
    raw = controller.get_my_position()
    did = int(controller.config["id"])
    return {**raw, "y": raw.get("y", 0.0) + _did_offset_y(did)}


def _target_common_to_local(controller: DroneController, target_common: Dict[str, float]) -> Dict[str, float]:
    did = int(controller.config["id"])
    return {
        "x": float(target_common["x"]),
        "y": float(target_common["y"]) - _did_offset_y(did),
        "z": float(target_common.get("z", -TAKEOFF_ALT_M)),
    }


def _clamp_rc(pwm: int, lo: int = 1100, hi: int = 1900) -> int:
    return max(lo, min(hi, int(pwm)))



def _distance_xy(a: Dict[str, float], b: Dict[str, float]) -> float:
    dx = float(a["x"]) - float(b["x"])
    dy = float(a["y"]) - float(b["y"])
    return (dx * dx + dy * dy) ** 0.5


def _send_neutral(controller: DroneController) -> None:
    if controller.worker:
        controller.worker.send_rc_override(
            RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, controller=controller
        )


def _move_towards_common_with_pid(
    controller: DroneController,
    target_common: Dict[str, float],
    roll_pid: PIDRegulator,
    pitch_pid: PIDRegulator,
    dt: float,
    distance_threshold: float,
    my_pos_common: Optional[Dict[str, float]] = None,
) -> None:
    """Move toward a common-frame target; uses my_pos_common for error and converts target to local frame."""
    if controller.worker is None:
        return
    if my_pos_common is None:
        my_pos_common = _my_position_common(controller)

    err_x = float(target_common["x"]) - float(my_pos_common["x"])
    err_y = float(target_common["y"]) - float(my_pos_common["y"])
    if abs(err_x) < ERROR_DEADBAND_M:
        err_x = 0.0
    if abs(err_y) < ERROR_DEADBAND_M:
        err_y = 0.0
    dist = (err_x * err_x + err_y * err_y) ** 0.5
    if dist < distance_threshold:
        _send_neutral(controller)
        roll_pid.reset()
        pitch_pid.reset()
        return

    pitch_output = pitch_pid.update(err_x, dt=dt)
    roll_output = roll_pid.update(err_y, dt=dt)

    # Pitch: forward/back, Roll: left/right.
    pitch = _clamp_rc(RC_NEUTRAL - int(pitch_output))
    roll = _clamp_rc(RC_NEUTRAL + int(roll_output))

    controller.worker.send_rc_override(roll, pitch, RC_NEUTRAL, RC_NEUTRAL, controller=controller)


def coordinate_exchange_loop(
    controllers: List[DroneController],
    duration: float,
    exchange_hz: float,
) -> None:
    """Exchange each drone's position in common NED frame and publish to visualizer if available."""
    global START_TIME
    loop_period = (1.0 / exchange_hz) if exchange_hz and exchange_hz > 0 else 0.0
    while True:
        if duration > 0 and (time.time() - START_TIME) >= duration:
            return
        t0 = time.time()

        positions_common: Dict[int, Dict[str, float]] = {}
        for c in controllers:
            did = int(c.config["id"])
            pos_raw = c.get_my_position()
            positions_common[did] = {**pos_raw, "y": pos_raw.get("y", 0.0) + _did_offset_y(did)}

        for c in controllers:
            my_id = int(c.config["id"])
            for did, pos in positions_common.items():
                if did != my_id:
                    c.update_other_drone_position(did, pos)

        if _publish_positions is not None:
            try:
                _publish_positions(positions_common, rates={"exchange_hz": exchange_hz})
            except Exception:
                pass

        if loop_period > 0:
            elapsed = time.time() - t0
            sleep_sec = loop_period - elapsed
            if sleep_sec > 0:
                time.sleep(sleep_sec)


def _segment_endpoints_common_from_anchors(
    anchor_left_common: Dict[str, float],
    anchor_right_common: Dict[str, float],
    length_m: float,
) -> Tuple[Dict[str, float], Dict[str, float]]:
    """
    Build fixed endpoints A,B for anchors.

    Key idea: spread along X by the requested segment length while keeping each
    anchor's current Y. This makes the segment direction use both X and Y, so
    internal drones align by true 2D Euclidean geometry (projection onto the
    anchor-to-anchor line), not by X alone.
    """
    half = 0.5 * float(length_m)
    a = {"x": -half, "y": float(anchor_left_common["y"]), "z": -TAKEOFF_ALT_M}
    b = {"x": +half, "y": float(anchor_right_common["y"]), "z": -TAKEOFF_ALT_M}
    return a, b


def _unit_vector_xy(a: Dict[str, float], b: Dict[str, float]) -> Tuple[float, float]:
    dx = float(b["x"]) - float(a["x"])
    dy = float(b["y"]) - float(a["y"])
    n = (dx * dx + dy * dy) ** 0.5
    if n <= 1e-9:
        return (1.0, 0.0)
    return (dx / n, dy / n)


def _projection_s(a: Dict[str, float], u: Tuple[float, float], p: Dict[str, float]) -> float:
    # Project onto segment direction in XY plane.
    px = float(p["x"]) - float(a["x"])
    py = float(p["y"]) - float(a["y"])
    return px * u[0] + py * u[1]


def endpoint_loop(
    controller: DroneController,
    target_common: Dict[str, float],
    duration: float,
) -> None:
    global START_TIME
    roll_pid = PIDRegulator(
        kp=ENDPOINT_Y_KP,
        ki=PID_KI,
        kd=PID_KD,
        integral_limit=PID_INTEGRAL_LIMIT,
        output_limit=ENDPOINT_OUTPUT_LIMIT,
    )
    pitch_pid = PIDRegulator(
        kp=ENDPOINT_X_KP,
        ki=PID_KI,
        kd=PID_KD,
        integral_limit=PID_INTEGRAL_LIMIT,
        output_limit=ENDPOINT_OUTPUT_LIMIT,
    )
    dt = 1.0 / CONTROL_HZ
    reached = False
    while True:
        if duration > 0 and (time.time() - START_TIME) >= duration:
            return
        my_common = _my_position_common(controller)
        if not reached:
            _move_towards_common_with_pid(
                controller,
                target_common,
                roll_pid,
                pitch_pid,
                dt=dt,
                distance_threshold=ENDPOINT_REACHED_THRESH_M,
                my_pos_common=my_common,
            )
            if _distance_xy(my_common, target_common) < ENDPOINT_REACHED_THRESH_M:
                reached = True
        else:
            _send_neutral(controller)
        time.sleep(dt)


def internal_chain_loop(
    controller: DroneController,
    endpoint_left_id: int,
    endpoint_right_id: int,
    endpoint_a_common: Dict[str, float],
    endpoint_b_common: Dict[str, float],
    duration: float,
) -> None:
    """Decentralized internal drone: orient by nearest neighbors along segment and go to their midpoint."""
    global START_TIME
    roll_pid = PIDRegulator(
        kp=INTERNAL_PID_KP,
        ki=INTERNAL_PID_KI,
        kd=INTERNAL_PID_KD,
        integral_limit=PID_INTEGRAL_LIMIT,
        output_limit=INTERNAL_PID_OUTPUT_LIMIT,
        derivative_alpha=INTERNAL_PID_DERIVATIVE_ALPHA,
    )
    pitch_pid = PIDRegulator(
        kp=INTERNAL_PID_KP,
        ki=INTERNAL_PID_KI,
        kd=INTERNAL_PID_KD,
        integral_limit=PID_INTEGRAL_LIMIT,
        output_limit=INTERNAL_PID_OUTPUT_LIMIT,
        derivative_alpha=INTERNAL_PID_DERIVATIVE_ALPHA,
    )
    dt = 1.0 / CONTROL_HZ
    u = _unit_vector_xy(endpoint_a_common, endpoint_b_common)

    my_id = int(controller.config["id"])
    prev_left_id: Optional[int] = None
    prev_right_id: Optional[int] = None
    prev_target_common: Optional[Dict[str, float]] = None
    while True:
        if duration > 0 and (time.time() - START_TIME) >= duration:
            return

        my_common = _my_position_common(controller)
        others = controller.get_other_drones_positions()  # already in common frame

        positions: Dict[int, Dict[str, float]] = {my_id: my_common}
        for did, pos in others.items():
            positions[int(did)] = pos

        # Wait until both anchors are visible.
        lp = positions.get(int(endpoint_left_id))
        rp = positions.get(int(endpoint_right_id))
        if lp is None or rp is None:
            # Not enough info yet; drift-minimize.
            _send_neutral(controller)
            time.sleep(dt)
            continue

        # Compute scalar projections along the segment direction.
        s_map: Dict[int, float] = {}
        for did, pos in positions.items():
            s_map[did] = _projection_s(endpoint_a_common, u, pos)

        # Stable sort to reduce jitter when projections are almost equal.
        ids_sorted = sorted(s_map.keys(), key=lambda k: (s_map[k], int(k)))
        if my_id not in ids_sorted or len(ids_sorted) < 3:
            _send_neutral(controller)
            time.sleep(dt)
            continue

        idx = ids_sorted.index(my_id)
        left_id = ids_sorted[idx - 1] if idx - 1 >= 0 else None
        right_id = ids_sorted[idx + 1] if idx + 1 < len(ids_sorted) else None

        # Neighbor hysteresis: if ordering is ambiguous, keep previous neighbors to avoid rapid flips.
        if prev_left_id is not None and prev_right_id is not None and left_id is not None and right_id is not None:
            # If current left/right differ from previous, only accept the change if the gap is clear.
            if left_id != prev_left_id or right_id != prev_right_id:
                s_me = s_map[my_id]
                s_left = s_map.get(left_id, s_me)
                s_right = s_map.get(right_id, s_me)
                # Ambiguous zone: very close neighbors / possible crossing.
                if (s_me - s_left) < NEIGHBOR_HYSTERESIS_M or (s_right - s_me) < NEIGHBOR_HYSTERESIS_M:
                    left_id, right_id = prev_left_id, prev_right_id

        if left_id is None or right_id is None:
            # Shouldn't happen for internal drones, but fall back to midpoint of endpoints.
            s_target = 0.5 * (_projection_s(endpoint_a_common, u, endpoint_a_common) + _projection_s(endpoint_a_common, u, endpoint_b_common))
        else:
            s_target = 0.5 * (s_map[left_id] + s_map[right_id])

        # Clamp target within endpoints [0, L] in projection space (avoid drifting past endpoints).
        s0 = _projection_s(endpoint_a_common, u, endpoint_a_common)
        s1 = _projection_s(endpoint_a_common, u, endpoint_b_common)
        s_min, s_max = (min(s0, s1), max(s0, s1))
        if s_target < s_min:
            s_target = s_min
        elif s_target > s_max:
            s_target = s_max

        target_common = {
            "x": float(endpoint_a_common["x"]) + u[0] * s_target,
            "y": float(endpoint_a_common["y"]) + u[1] * s_target,
            "z": -TAKEOFF_ALT_M,
        }

        # Smooth target to avoid bang-bang chasing when neighbors jitter.
        if prev_target_common is not None:
            a = TARGET_SMOOTHING_ALPHA
            target_common = {
                "x": a * float(prev_target_common["x"]) + (1.0 - a) * float(target_common["x"]),
                "y": a * float(prev_target_common["y"]) + (1.0 - a) * float(target_common["y"]),
                "z": -TAKEOFF_ALT_M,
            }
        prev_target_common = dict(target_common)
        prev_left_id, prev_right_id = left_id, right_id

        # --- Velocity damping along the segment (projection space) ---
        # Important: apply damping by *shifting the target along the segment*, not by sending
        # a separate RC command. This keeps the desired point strictly on the segment line,
        # so drones converge to a single straight line instead of stabilizing near it.
        s_me = float(s_map[my_id])
        v_s = 0.0
        if controller.velocity_monitor is not None:
            vel = controller.velocity_monitor.get_velocity()
            vx = float(vel.get("vx", 0.0))
            vy = float(vel.get("vy", 0.0))
            v_s = vx * float(u[0]) + vy * float(u[1])
        e_s = float(s_target) - s_me
        if abs(e_s) < INTERNAL_S_DEADBAND_M:
            e_s = 0.0
        # PD in projection space -> equivalent "desired delta s"
        u_pwm = INTERNAL_S_KP_PWM_PER_M * e_s - INTERNAL_S_KV_PWM_PER_MPS * v_s
        u_pwm = max(-INTERNAL_S_OUTPUT_LIMIT_PWM, min(INTERNAL_S_OUTPUT_LIMIT_PWM, u_pwm))
        # Convert to meters shift along segment (avoid division by 0)
        if INTERNAL_S_KP_PWM_PER_M > 1e-6:
            delta_s = float(u_pwm) / float(INTERNAL_S_KP_PWM_PER_M)
        else:
            delta_s = 0.0
        s_target_damped = s_me + delta_s
        # Clamp again to segment bounds
        if s_target_damped < s_min:
            s_target_damped = s_min
        elif s_target_damped > s_max:
            s_target_damped = s_max
        target_common = {
            "x": float(endpoint_a_common["x"]) + u[0] * s_target_damped,
            "y": float(endpoint_a_common["y"]) + u[1] * s_target_damped,
            "z": -TAKEOFF_ALT_M,
        }

        _move_towards_common_with_pid(
            controller,
            target_common,
            roll_pid,
            pitch_pid,
            dt=dt,
            distance_threshold=INTERNAL_REACHED_THRESH_M,
            my_pos_common=my_common,
        )
        time.sleep(dt)


def initialize_drone_parallel(
    controller: DroneController,
    init_barrier: threading.Barrier,
    barrier_timeout_sec: float = 45.0,
) -> None:
    try:
        controller.connect()
        controller.initialize(INIT_STEPS)
        controller.start_rc_keepalive()
        init_barrier.wait(timeout=barrier_timeout_sec)
    except Exception:
        # Make sure other threads un-block if one init fails.
        try:
            init_barrier.abort()
        except Exception:
            pass
        logger.exception("Drone init failed")
        return


def main() -> None:
    import argparse

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    parser = argparse.ArgumentParser(description="Линейная цепь (linear_chain)")
    parser.add_argument("--drones", type=int, default=4, help="Number of drones (>=4)")
    parser.add_argument("--duration", type=float, default=0, help="Experiment duration (s); 0 = no limit")
    parser.add_argument("--experiment-dir", type=str, default=None, help="Unused (kept for launcher compatibility)")
    parser.add_argument("--segment-length", type=float, default=SEGMENT_LENGTH_M, help="Segment length in meters")
    parser.add_argument(
        "--exchange-hz",
        type=float,
        default=50.0,
        help="Coordinate exchange loop rate (Hz); match SITL position stream (default 50)",
    )
    args = parser.parse_args()

    num_drones = int(args.drones)
    if num_drones < 4:
        logger.warning("linear_chain requires >=4 drones; using 4 instead of %s", num_drones)
        num_drones = 4

    drones_config = [
        {"id": i + 1, "udp_port": 14551 + i * 10, "role": "chain"}
        for i in range(num_drones)
    ]
    controllers: List[DroneController] = [
        DroneController(cfg, logging_enabled=False) for cfg in drones_config
    ]

    init_barrier = threading.Barrier(len(controllers) + 1)
    for c in controllers:
        threading.Thread(
            target=initialize_drone_parallel, args=(c, init_barrier), daemon=False
        ).start()
    try:
        init_barrier.wait(timeout=45)
    except threading.BrokenBarrierError:
        for c in controllers:
            try:
                c.stop()
            except Exception:
                pass
        return

    time.sleep(2)
    global START_TIME
    START_TIME = time.time()

    exchange_hz = max(0.0, float(getattr(args, "exchange_hz", 50.0)))
    threading.Thread(
        target=coordinate_exchange_loop,
        args=(controllers, float(args.duration), exchange_hz),
        daemon=True,
    ).start()
    time.sleep(1)

    ids = sorted(int(c.config["id"]) for c in controllers)
    endpoint_left_id = min(ids)
    endpoint_right_id = max(ids)
    # Compute fixed segment endpoints using anchors' post-takeoff Y (stable), but fixed X spread.
    anchor_left = next(c for c in controllers if int(c.config["id"]) == endpoint_left_id)
    anchor_right = next(c for c in controllers if int(c.config["id"]) == endpoint_right_id)
    time.sleep(2.0)
    endpoint_a_common, endpoint_b_common = _segment_endpoints_common_from_anchors(
        _my_position_common(anchor_left),
        _my_position_common(anchor_right),
        float(getattr(args, "segment_length", SEGMENT_LENGTH_M)),
    )

    threads: List[threading.Thread] = []
    for c in controllers:
        did = int(c.config["id"])
        if did == endpoint_left_id:
            t = threading.Thread(
                target=endpoint_loop,
                args=(c, endpoint_a_common, float(args.duration)),
                daemon=False,
            )
        elif did == endpoint_right_id:
            t = threading.Thread(
                target=endpoint_loop,
                args=(c, endpoint_b_common, float(args.duration)),
                daemon=False,
            )
        else:
            t = threading.Thread(
                target=internal_chain_loop,
                args=(
                    c,
                    int(endpoint_left_id),
                    int(endpoint_right_id),
                    endpoint_a_common,
                    endpoint_b_common,
                    float(args.duration),
                ),
                daemon=False,
            )
        t.start()
        threads.append(t)

    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        pass
    finally:
        for c in controllers:
            try:
                c.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()


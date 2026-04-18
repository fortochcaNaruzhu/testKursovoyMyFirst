"""
Линейная цепь v3 (linear_chain3): те же якоря, обмен координатами и движение концов, что в linear_chain2,
но внутренние дроны используют только локальный закон коридора в базисе текущего отрезка AB (без явного
геометрического e_cross/e_along).

Базис: u — единичный вектор вдоль AB, n — перпендикуляр в плоскости XY (слева от u).
Все видимые объекты в радиусе r_vis, включая концевые дроны, считаются равноправными соседями;
относительные векторы проецируются на (along, cross). Ближайшие расстояния в четырёх полупространствах
и нелинейность tanh — как в corridor_sweeping; вдоль цепи виртуальный зазор 0, поперёк — w при отсутствии
соседа с той стороны. В SITL v_along = v_cross = 0. Комбинированная горизонтальная команда в NED:
sigma_a * u + sigma_c * n, затем поворот в корпус, демпфирование скорости и RC как в linear_chain2.

Управление: RC override. Pitch/roll в плоскости корпуса; при смещении осей проверьте YAW_SIGN и
RC_OVERRIDE_SWAP_ROLL_PITCH.
"""

from __future__ import annotations

import logging
import math
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

START_TIME = 0.0

TAKEOFF_ALT_M = 1.0
SEGMENT_LENGTH_M = 8.0
ENDPOINT_REACHED_THRESH_M = 0.5

CONTROL_HZ = 18.0
ERROR_DEADBAND_M = 0.12

ENDPOINT_X_KP = 560.0
ENDPOINT_Y_KP = 200.0
ENDPOINT_OUTPUT_LIMIT = 220.0

PID_KI = 0.0
PID_KD = 0.0
PID_INTEGRAL_LIMIT = 100.0

# --- Corridor-style internal law in segment frame (AB), no explicit e_along/e_cross geometry ---
R_VIS_M = 50.0
W_CROSS_M = 0.6
SIGMA_DEADBAND = 0.03
INTERNAL_RC_KP = 68.0
INTERNAL_RC_MAX_PWM = 118
INTERNAL_RC_MIN_STEP_PWM = 22
INTERNAL_RC_SLEW_PWM_PER_CYCLE = 24
INTERNAL_BODY_V_DAMP = 0.42
INTERNAL_BODY_V_DAMP_CLAMP = 0.95

YAW_SIGN = -1.0
INTERNAL_RC_KP_ROLL_MULT = 1.22
RC_OVERRIDE_SWAP_ROLL_PITCH = False

INIT_STEPS = [
    {"type": "set_mode", "mode_id": 4},
    {"type": "sleep", "sec": 0.5},
    {"type": "arm"},
    {"type": "sleep", "sec": 2},
    {"type": "takeoff"},
    {"type": "sleep", "sec": 5},
    {"type": "set_mode", "mode_id": 16},
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


def _distance_xy(a: Dict[str, float], b: Dict[str, float]) -> float:
    dx = float(a["x"]) - float(b["x"])
    dy = float(a["y"]) - float(b["y"])
    return (dx * dx + dy * dy) ** 0.5


def _clamp_rc(pwm: int, lo: int = 1100, hi: int = 1900) -> int:
    return max(lo, min(hi, int(pwm)))


def _yaw_rad(controller: DroneController) -> float:
    cm = getattr(controller, "coords_monitor", None)
    if cm is None:
        return 0.0
    try:
        att = cm.get_attitude()
        return YAW_SIGN * float(att.get("rz", 0.0))
    except Exception:
        return 0.0


def _velocity_xy_ned(controller: DroneController) -> Tuple[float, float]:
    vm = getattr(controller, "velocity_monitor", None)
    if vm is None:
        return 0.0, 0.0
    try:
        vel = vm.get_velocity()
        return float(vel.get("vx", 0.0)), float(vel.get("vy", 0.0))
    except Exception:
        return 0.0, 0.0


def _ned_to_body_forward_right(en: float, ee: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    forward = en * c + ee * s
    right = -en * s + ee * c
    return forward, right


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

    yaw = _yaw_rad(controller)
    ef, er = _ned_to_body_forward_right(err_x, err_y, yaw)
    pitch_output = pitch_pid.update(ef, dt=dt)
    roll_output = roll_pid.update(er, dt=dt)
    pitch = _clamp_rc(RC_NEUTRAL - int(pitch_output))
    roll = _clamp_rc(RC_NEUTRAL + int(roll_output))
    if RC_OVERRIDE_SWAP_ROLL_PITCH:
        controller.worker.send_rc_override(
            pitch, roll, RC_NEUTRAL, RC_NEUTRAL, controller=controller
        )
    else:
        controller.worker.send_rc_override(
            roll, pitch, RC_NEUTRAL, RC_NEUTRAL, controller=controller
        )


def coordinate_exchange_loop(
    controllers: List[DroneController],
    duration: float,
    exchange_hz: float,
) -> None:
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


def _unit_vector_xy(a: Dict[str, float], b: Dict[str, float]) -> Tuple[float, float]:
    dx = float(b["x"]) - float(a["x"])
    dy = float(b["y"]) - float(a["y"])
    n = (dx * dx + dy * dy) ** 0.5
    if n <= 1e-9:
        return (1.0, 0.0)
    return (dx / n, dy / n)


def _anchor_targets_from_initial_segment(
    anchor_left_common: Dict[str, float],
    anchor_right_common: Dict[str, float],
    length_m: float,
) -> Tuple[Dict[str, float], Dict[str, float]]:
    z = -TAKEOFF_ALT_M
    lx = float(anchor_left_common["x"])
    ly = float(anchor_left_common["y"])
    rx = float(anchor_right_common["x"])
    ry = float(anchor_right_common["y"])
    half = 0.5 * float(length_m)
    a = {"x": lx, "y": ly - half, "z": z}
    b = {"x": rx + half, "y": ry, "z": z}
    return a, b


def _normal_left_xy(u: Tuple[float, float]) -> Tuple[float, float]:
    return (-float(u[1]), float(u[0]))


def _xi(g_val: float) -> float:
    return math.tanh(g_val)


def _g(dist: float, gap: float, has_peer: bool) -> float:
    return dist if has_peer else gap


def _nearest_along_cross(
    peers_along: List[float], peers_cross: List[float]
) -> Tuple[List[float], List[bool]]:
    min_ap, min_cp = float("inf"), float("inf")
    min_am, min_cm = float("inf"), float("inf")

    for a, c in zip(peers_along, peers_cross):
        if a >= 0:
            if a < min_ap:
                min_ap = a
        else:
            na = -a
            if na < min_am:
                min_am = na
        if c >= 0:
            if c < min_cp:
                min_cp = c
        else:
            nc = -c
            if nc < min_cm:
                min_cm = nc

    dists = [min_ap, min_cp, min_am, min_cm]
    flags = [d < float("inf") for d in dists]
    for i in range(4):
        if not flags[i]:
            dists[i] = float("inf")
    return dists, flags


def _sigmas_segment_frame(
    distances: List[float],
    flags: List[bool],
    v_along: float,
    v_cross: float,
    w_cross: float,
) -> Tuple[float, float]:
    d_ap, d_cp, d_am, d_cm = distances
    f_ap, f_cp, f_am, f_cm = flags
    sigma_a = v_along - _xi(_g(d_ap, 0.0, f_ap)) + _xi(_g(d_am, 0.0, f_am))
    sigma_c = v_cross - _xi(_g(d_cp, w_cross, f_cp)) + _xi(_g(d_cm, w_cross, f_cm))
    return sigma_a, sigma_c


def _clamp_pwm_delta(delta: int, lim: int) -> int:
    if delta > lim:
        return lim
    if delta < -lim:
        return -lim
    return delta


def _clamp_m(x: float, lim: float) -> float:
    if x > lim:
        return lim
    if x < -lim:
        return -lim
    return x


def _pwm_axis(error: float, kp: float, lim: int, min_step: int) -> int:
    if abs(error) < 1e-6:
        return 0
    v = int(round(kp * error))
    if v == 0 and abs(error) > 1e-5:
        v = min_step if error > 0 else -min_step
    return _clamp_pwm_delta(v, lim)


def _slew_int(cur: int, target: int, max_step: int) -> int:
    if target > cur + max_step:
        return cur + max_step
    if target < cur - max_step:
        return cur - max_step
    return target


def internal_chain_corridor_loop(
    controller: DroneController,
    endpoint_left_id: int,
    endpoint_right_id: int,
    duration: float,
    r_vis: float,
    w_cross: float,
) -> None:
    """
    Внутренний дрон: чистый закон коридора в базисе AB; концы — такие же видимые соседи, как остальные.
    Поперечный дисбаланс (плюс/минус по n) задаётся sigma_c и виртуальным зазором w_cross при пустом квадранте.
    """
    global START_TIME
    dt = 1.0 / CONTROL_HZ

    my_id = int(controller.config["id"])
    prev_pd = 0
    prev_rd = 0

    while True:
        if duration > 0 and (time.time() - START_TIME) >= duration:
            return

        my_common = _my_position_common(controller)
        others = controller.get_other_drones_positions()

        positions: Dict[int, Dict[str, float]] = {my_id: my_common}
        for did, pos in others.items():
            positions[int(did)] = pos

        pos_l = positions.get(int(endpoint_left_id))
        pos_r = positions.get(int(endpoint_right_id))
        if pos_l is None or pos_r is None:
            prev_pd, prev_rd = 0, 0
            _send_neutral(controller)
            time.sleep(dt)
            continue

        u = _unit_vector_xy(pos_l, pos_r)
        n = _normal_left_xy(u)

        mx, my = float(my_common["x"]), float(my_common["y"])
        peers_along: List[float] = []
        peers_cross: List[float] = []

        for did, pos in positions.items():
            if int(did) == my_id:
                continue
            ox = float(pos["x"]) - mx
            oy = float(pos["y"]) - my
            d = (ox * ox + oy * oy) ** 0.5
            if d > r_vis or d < 1e-6:
                continue
            along = ox * u[0] + oy * u[1]
            cross = ox * n[0] + oy * n[1]
            peers_along.append(along)
            peers_cross.append(cross)

        v_along = 0.0
        v_cross = 0.0

        dists, flgs = _nearest_along_cross(peers_along, peers_cross)
        sigma_a, sigma_c = _sigmas_segment_frame(
            dists, flgs, v_along, v_cross, w_cross=w_cross
        )

        if abs(sigma_a) < SIGMA_DEADBAND and abs(sigma_c) < SIGMA_DEADBAND:
            prev_pd, prev_rd = 0, 0
            _send_neutral(controller)
            time.sleep(dt)
            continue

        comb_x = sigma_a * u[0] + sigma_c * n[0]
        comb_y = sigma_a * u[1] + sigma_c * n[1]
        yaw = _yaw_rad(controller)
        fwd, rig = _ned_to_body_forward_right(comb_x, comb_y, yaw)
        vx, vy = _velocity_xy_ned(controller)
        vf, vr = _ned_to_body_forward_right(vx, vy, yaw)
        damp_f = _clamp_m(INTERNAL_BODY_V_DAMP * vf, INTERNAL_BODY_V_DAMP_CLAMP)
        damp_r = _clamp_m(INTERNAL_BODY_V_DAMP * vr, INTERNAL_BODY_V_DAMP_CLAMP)
        fwd -= damp_f
        rig -= damp_r
        kp_r = INTERNAL_RC_KP * INTERNAL_RC_KP_ROLL_MULT
        pd_tgt = _pwm_axis(fwd, INTERNAL_RC_KP, INTERNAL_RC_MAX_PWM, INTERNAL_RC_MIN_STEP_PWM)
        rd_tgt = _pwm_axis(rig, kp_r, INTERNAL_RC_MAX_PWM, INTERNAL_RC_MIN_STEP_PWM)
        pd = _slew_int(prev_pd, pd_tgt, INTERNAL_RC_SLEW_PWM_PER_CYCLE)
        rd = _slew_int(prev_rd, rd_tgt, INTERNAL_RC_SLEW_PWM_PER_CYCLE)
        prev_pd, prev_rd = pd, rd
        if pd == 0 and rd == 0:
            prev_pd, prev_rd = 0, 0
            _send_neutral(controller)
            time.sleep(dt)
            continue

        pitch = _clamp_rc(RC_NEUTRAL - pd)
        roll = _clamp_rc(RC_NEUTRAL + rd)
        if controller.worker:
            if RC_OVERRIDE_SWAP_ROLL_PITCH:
                controller.worker.send_rc_override(
                    pitch, roll, RC_NEUTRAL, RC_NEUTRAL, controller=controller
                )
            else:
                controller.worker.send_rc_override(
                    roll, pitch, RC_NEUTRAL, RC_NEUTRAL, controller=controller
                )
        time.sleep(dt)


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
        try:
            init_barrier.abort()
        except Exception:
            pass
        logger.exception("Drone init failed")
        return


def main() -> None:
    import argparse

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    parser = argparse.ArgumentParser(
        description="Линейная цепь v3 — коридорный закон в базисе AB (linear_chain3)"
    )
    parser.add_argument("--drones", type=int, default=4, help="Number of drones (>=4)")
    parser.add_argument("--duration", type=float, default=0, help="Experiment duration (s); 0 = no limit")
    parser.add_argument("--experiment-dir", type=str, default=None, help="Unused (launcher compatibility)")
    parser.add_argument(
        "--segment-length",
        type=float,
        default=SEGMENT_LENGTH_M,
        help="Anchor target stretch (m): left endpoint -Y by half, right endpoint +X by half",
    )
    parser.add_argument(
        "--exchange-hz",
        type=float,
        default=50.0,
        help="Coordinate exchange rate (Hz)",
    )
    parser.add_argument(
        "--r-vis",
        type=float,
        default=R_VIS_M,
        help="Visibility radius for peers (m)",
    )
    parser.add_argument(
        "--w-cross",
        type=float,
        default=W_CROSS_M,
        help="Virtual cross-track gap when no peer in that half-space (corridor w)",
    )
    args = parser.parse_args()

    num_drones = int(args.drones)
    if num_drones < 4:
        logger.warning("linear_chain3 requires >=4 drones; using 4 instead of %s", num_drones)
        num_drones = 4

    drones_config = [
        {"id": i + 1, "udp_port": 14551 + i * 10, "role": "chain3"}
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
    anchor_left = next(c for c in controllers if int(c.config["id"]) == endpoint_left_id)
    anchor_right = next(c for c in controllers if int(c.config["id"]) == endpoint_right_id)
    time.sleep(2.0)
    a_target, b_target = _anchor_targets_from_initial_segment(
        _my_position_common(anchor_left),
        _my_position_common(anchor_right),
        float(getattr(args, "segment_length", SEGMENT_LENGTH_M)),
    )

    r_vis = float(args.r_vis)
    w_cross = float(args.w_cross)

    threads: List[threading.Thread] = []
    for c in controllers:
        did = int(c.config["id"])
        if did == endpoint_left_id:
            t = threading.Thread(
                target=endpoint_loop,
                args=(c, a_target, float(args.duration)),
                daemon=False,
            )
        elif did == endpoint_right_id:
            t = threading.Thread(
                target=endpoint_loop,
                args=(c, b_target, float(args.duration)),
                daemon=False,
            )
        else:
            t = threading.Thread(
                target=internal_chain_corridor_loop,
                args=(
                    c,
                    int(endpoint_left_id),
                    int(endpoint_right_id),
                    float(args.duration),
                    r_vis,
                    w_cross,
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

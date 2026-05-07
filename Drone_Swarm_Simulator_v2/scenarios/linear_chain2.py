"""
Линейная цепь v2 (linear_chain2): те же якоря и отрезок, что в linear_chain,
но внутренние дроны используют распределённый закон из Anatoliy/corridor_sweeping:
ближайшие «соседи» в четырёх полупространствах, нелинейность tanh и bang-bang по знаку.

Оси для закона не мировые X/Y, а локальный базис отрезка AB:
  u — единичный вектор вдоль цепи, n — перпендикуляр в плоскости XY (слева от u).
Так цепь может быть наклонной, как в linear_chain.

Управление: RC override. Внутренние: закон Anatoliy (tanh по квадрантам, v=0 в SITL) +
пропорциональные стики; геометрия в common NED как +e_along*u и −e_cross*n.

Важно: pitch/roll — в плоскости корпуса, comb — в NED. Перед стиками: поворот по yaw из телеметрии (SIM_STATE → worker.get_attitude).
Если по графику едет только North, а East «мёртвый» — проверь YAW_SIGN (часто в SITL нужен −1)
и RC_OVERRIDE_SWAP_ROLL_PITCH под свою RCMAP.
"""

from __future__ import annotations

import csv
import json
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

# --- Anatoliy-style internal law (corridor_sweeping/main.py), segment frame ---
R_VIS_M = 50.0
# «Виртуальный зазор» поперёк цепи, когда с той стороны нет видимого соседа (как w в corridor).
W_CROSS_M = 0.6
# Мёртвая зона по sigma (Anatoliy); геометрия цепи задаётся отдельно и может тянуть сильнее.
SIGMA_DEADBAND = 0.03
# Вклад Anatoliy в comb (0..1); остальное — геометрия как в linear_chain.
ANATOLIY_COMB_WEIGHT = 0.22
# Пропорциональный закон: delta_pwm ≈ KP * (fwd/rig после демпфирования скорости).
INTERNAL_RC_KP = 68.0
INTERNAL_RC_MAX_PWM = 118
INTERNAL_RC_MIN_STEP_PWM = 22
# Ограничение |ΔPWM| за один такт (сглаживает переуправление и разгон).
INTERNAL_RC_SLEW_PWM_PER_CYCLE = 24
# Демпфирование по NED vx,vy → в body: уменьшает команды, если уже летим в ту же сторону.
INTERNAL_BODY_V_DAMP = 0.42
INTERNAL_BODY_V_DAMP_CLAMP = 0.95
# Поправки равномерности (умеренные — меньше перерегулирование у средних).
SPACING_ALONG_K = 0.36
SPACING_CROSS_K = 0.4
SPACING_E_ALONG_CLAMP_M = 2.2
SPACING_E_CROSS_CLAMP_M = 2.2
SPACING_E_DEADBAND_M = 0.065

# Знак yaw из телеметрии относительно NED (множитель на rz). −1 часто совпадает с SITL ArduPilot.
YAW_SIGN = -1.0
# Усиление канала roll (East при носе на север): POS_HOLD часто слабее реагирует на крен.
INTERNAL_RC_KP_ROLL_MULT = 1.22
# Если восток/запад всё ещё через pitch — поменять местами chan1/chan2 в override (RCMAP).
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
    """Рыскание из worker attitude / SIM_STATE (rad), с учётом YAW_SIGN."""
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
    """
    Ошибка/команда в горизонтальной NED (x=North, y=East) → вперёд и вправо по корпусу.
    Совпадает с linear_chain при yaw=0: вперёд=North, вправо=East.
    """
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


def _axis_interval_and_perp_distance_m(
    positions: Dict[int, Dict[str, float]],
    endpoint_left_id: int,
    endpoint_right_id: int,
) -> Optional[Tuple[Dict[int, float], Dict[int, float]]]:
    """
    Target axis = line through current endpoint positions (left → right).
    Returns (signed_along_m, perp_distance_m) per drone id.
    Along: projection (p - A)·u with A = left endpoint, u unit toward right.
    Perp distance: |(p - A)·n| with n left normal to u in XY (NED).
    """
    pos_l = positions.get(int(endpoint_left_id))
    pos_r = positions.get(int(endpoint_right_id))
    if pos_l is None or pos_r is None:
        return None
    u = _unit_vector_xy(pos_l, pos_r)
    n = _normal_left_xy(u)
    ax = float(pos_l["x"])
    ay = float(pos_l["y"])
    along: Dict[int, float] = {}
    dist: Dict[int, float] = {}
    for did, p in positions.items():
        iid = int(did)
        px = float(p["x"])
        py = float(p["y"])
        along[iid] = (px - ax) * u[0] + (py - ay) * u[1]
        cross = (px - ax) * n[0] + (py - ay) * n[1]
        dist[iid] = abs(cross)
    return along, dist


def _write_experiment_metadata(experiment_dir: str, metadata: Dict[str, object]) -> None:
    path = os.path.join(experiment_dir, "metadata.json")
    try:
        from core.logging.csv_logger import write_metadata as _write_meta

        _write_meta(
            experiment_dir,
            float(metadata.get("duration_s", 0.0)),
            0.0,
            int(metadata.get("num_drones", 0)),
            str(metadata.get("scenario", "linear_chain2")),
            extra={
                k: v
                for k, v in metadata.items()
                if k not in {"duration_s", "num_drones", "scenario"}
            },
        )
        return
    except Exception:
        pass
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2, sort_keys=True)
    except Exception:
        logger.exception("Failed to write metadata.json")


def _finalize_axis_experiment_artifacts(
    experiment_dir: str,
    drone_ids: List[int],
    metrics_rows: List[Tuple[float, Dict[int, float], Dict[int, float]]],
    metadata: Dict[str, object],
) -> None:
    if not metrics_rows:
        logger.warning("No axis metric samples recorded; skipping CSV/plots.")
        return
    os.makedirs(experiment_dir, exist_ok=True)
    _write_experiment_metadata(experiment_dir, metadata)

    id_cols = [f"d{did}" for did in drone_ids]
    intervals_path = os.path.join(experiment_dir, "axis_intervals.csv")
    error_path = os.path.join(experiment_dir, "axis_error.csv")
    try:
        with open(intervals_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s"] + id_cols)
            for t_s, along, _dist in metrics_rows:
                w.writerow([f"{t_s:.6f}"] + [f"{along.get(did, float('nan')):.6f}" for did in drone_ids])
        with open(error_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["time_s"] + id_cols)
            for t_s, _along, dist in metrics_rows:
                w.writerow([f"{t_s:.6f}"] + [f"{dist.get(did, float('nan')):.6f}" for did in drone_ids])
    except Exception:
        logger.exception("Failed to write axis metric CSV files")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        times = [r[0] for r in metrics_rows]
        fig, ax = plt.subplots(figsize=(10, 5))
        for did in drone_ids:
            ys = [row[1].get(did, float("nan")) for row in metrics_rows]
            ax.plot(times, ys, label=f"drone {did}", linewidth=1.0)
        ax.set_xlabel("t")
        ax.set_ylabel("d")
        ax.set_title("Intervals along target axis for all agents")
        ax.grid(True, alpha=0.3)
        if len(drone_ids) <= 12:
            ax.legend(loc="best", fontsize=8, ncol=2 if len(drone_ids) > 6 else 1)
        fig.tight_layout()
        fig.savefig(
            os.path.join(experiment_dir, "linear_chain2_intervals.jpg"),
            format="jpg",
            dpi=120,
        )
        plt.close(fig)

        fig2, ax2 = plt.subplots(figsize=(10, 5))
        for did in drone_ids:
            ys = [row[2].get(did, float("nan")) for row in metrics_rows]
            ax2.plot(times, ys, label=f"drone {did}", linewidth=1.0)
        ax2.set_xlabel("t")
        ax2.set_ylabel("d")
        ax2.set_title("Distance from target axis for all agents")
        ax2.grid(True, alpha=0.3)
        if len(drone_ids) <= 12:
            ax2.legend(loc="best", fontsize=8, ncol=2 if len(drone_ids) > 6 else 1)
        fig2.tight_layout()
        fig2.savefig(
            os.path.join(experiment_dir, "linear_chain2_axis_err.jpg"),
            format="jpg",
            dpi=120,
        )
        plt.close(fig2)
    except Exception as e:
        logger.warning("matplotlib plot export failed (%s); CSVs are still saved.", e)


def coordinate_exchange_loop(
    controllers: List[DroneController],
    duration: float,
    exchange_hz: float,
    endpoint_left_id: int,
    endpoint_right_id: int,
    metrics_lock: Optional[threading.Lock] = None,
    metrics_rows: Optional[List[Tuple[float, Dict[int, float], Dict[int, float]]]] = None,
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

        if metrics_rows is not None and metrics_lock is not None:
            axis_m = _axis_interval_and_perp_distance_m(
                positions_common, endpoint_left_id, endpoint_right_id
            )
            if axis_m is not None:
                t_rel = time.time() - START_TIME
                along_m, dist_m = axis_m
                with metrics_lock:
                    metrics_rows.append((t_rel, along_m, dist_m))

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
    """A_target, B_target: left anchor moves along -Y, right along +X, each by half of length_m."""
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
    """Перпендикуляр к u в XY (поворот +90° в плоскости NED: влево от направления u)."""
    return (-float(u[1]), float(u[0]))


def _projection_s(a: Dict[str, float], u: Tuple[float, float], p: Dict[str, float]) -> float:
    px = float(p["x"]) - float(a["x"])
    py = float(p["y"]) - float(a["y"])
    return px * u[0] + py * u[1]


def _xi(g_val: float) -> float:
    return math.tanh(g_val)


def _g(dist: float, gap: float, has_peer: bool) -> float:
    return dist if has_peer else gap


def _nearest_along_cross(
    peers_along: List[float], peers_cross: List[float]
) -> Tuple[List[float], List[bool]]:
    """
    Аналог get_nearest_distances(corridor_sweeping): четыре квадранта по осям (along, cross).
    Возвращает [d_a_plus, d_c_plus, d_a_minus, d_c_minus] и флаги наличия соседа.
    """
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
    """sigma_along, sigma_cross — как в corridor_sweeping control_force (оси along/cross)."""
    d_ap, d_cp, d_am, d_cm = distances
    f_ap, f_cp, f_am, f_cm = flags
    sigma_a = v_along - _xi(_g(d_ap, 0.0, f_ap)) + _xi(_g(d_am, 0.0, f_am))
    sigma_c = v_cross - _xi(_g(d_cp, w_cross, f_cp)) + _xi(_g(d_cm, w_cross, f_cm))
    return sigma_a, sigma_c


def _spacing_errors_projection(
    positions: Dict[int, Dict[str, float]],
    my_id: int,
    endpoint_a: Dict[str, float],
    u: Tuple[float, float],
    n: Tuple[float, float],
) -> Tuple[float, float]:
    """
    e_along: ошибка «середина между соседями по s» (м); 0 если соседей по порядку нет.
    e_cross: signed cross-track к прямой через A вдоль u (м).
    """
    s_map: Dict[int, float] = {}
    for did, pos in positions.items():
        s_map[int(did)] = _projection_s(endpoint_a, u, pos)
    ids_sorted = sorted(s_map.keys(), key=lambda k: (s_map[k], int(k)))
    if my_id not in ids_sorted:
        return 0.0, 0.0
    idx = ids_sorted.index(my_id)
    e_along = 0.0
    if idx > 0 and idx + 1 < len(ids_sorted):
        lid = ids_sorted[idx - 1]
        rid = ids_sorted[idx + 1]
        s_me = s_map[my_id]
        e_along = 0.5 * (s_map[lid] + s_map[rid]) - s_me
    mx = float(positions[my_id]["x"])
    my = float(positions[my_id]["y"])
    ax = float(endpoint_a["x"])
    ay = float(endpoint_a["y"])
    e_cross = (mx - ax) * n[0] + (my - ay) * n[1]
    return e_along, e_cross


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
    """PWM по одной оси; при мелкой ошибке не даём 0, чтобы не стоять на нейтрали."""
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


def internal_chain_anatoliy_loop(
    controller: DroneController,
    endpoint_left_id: int,
    endpoint_right_id: int,
    duration: float,
    r_vis: float,
    w_cross: float,
) -> None:
    """Внутренний дрон: закон corridor_sweeping в базисе отрезка AB + RC override."""
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
        endpoint_a = pos_l

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

        if len(peers_along) < 2:
            prev_pd, prev_rd = 0, 0
            _send_neutral(controller)
            time.sleep(dt)
            continue

        # В SITL скорость из стрима часто шумная / не гарантирована в earth NED — закон коридора
        # для point-mass опирался на v; здесь оставляем v=0, иначе sigma «плывёт».
        v_along = 0.0
        v_cross = 0.0

        dists, flgs = _nearest_along_cross(peers_along, peers_cross)
        sigma_a, sigma_c = _sigmas_segment_frame(
            dists, flgs, v_along, v_cross, w_cross=w_cross
        )
        e_along, e_cross = _spacing_errors_projection(
            positions, my_id, endpoint_a, u, n
        )
        e_along = _clamp_m(e_along, SPACING_E_ALONG_CLAMP_M)
        e_cross = _clamp_m(e_cross, SPACING_E_CROSS_CLAMP_M)

        spacing_pull = (
            abs(e_along) > SPACING_E_DEADBAND_M or abs(e_cross) > SPACING_E_DEADBAND_M
        )
        if (
            not spacing_pull
            and abs(sigma_a) < SIGMA_DEADBAND
            and abs(sigma_c) < SIGMA_DEADBAND
        ):
            prev_pd, prev_rd = 0, 0
            _send_neutral(controller)
            time.sleep(dt)
            continue

        # Геометрия цепи (как linear_chain): главный вклад в comb.
        gx = SPACING_ALONG_K * e_along * u[0] - SPACING_CROSS_K * e_cross * n[0]
        gy = SPACING_ALONG_K * e_along * u[1] - SPACING_CROSS_K * e_cross * n[1]
        # Anatoliy — вспомогательный, уменьшенный (иначе мешает лечь на одну прямую).
        ax = sigma_a * u[0] + sigma_c * n[0]
        ay = sigma_a * u[1] + sigma_c * n[1]
        comb_x = gx + ANATOLIY_COMB_WEIGHT * ax
        comb_y = gy + ANATOLIY_COMB_WEIGHT * ay
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
    parser = argparse.ArgumentParser(description="Линейная цепь v2 — закон Anatoliy (linear_chain2)")
    parser.add_argument("--drones", type=int, default=4, help="Number of drones (>=4)")
    parser.add_argument("--duration", type=float, default=0, help="Experiment duration (s); 0 = no limit")
    parser.add_argument(
        "--experiment-dir",
        type=str,
        default=None,
        help="Directory for axis_intervals.csv, axis_error.csv, plots, metadata (default when duration>0: experiments/...)",
    )
    parser.add_argument(
        "--run-id",
        type=str,
        default=None,
        help="When --experiment-dir omitted: use experiments/exp_<run-id> (launcher compatibility)",
    )
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
        help="Visibility radius for peers (m), Anatoliy R_vis",
    )
    parser.add_argument(
        "--w-cross",
        type=float,
        default=W_CROSS_M,
        help="Virtual cross-track gap when no peer (corridor w)",
    )
    args = parser.parse_args()

    num_drones = int(args.drones)
    if num_drones < 4:
        logger.warning("linear_chain2 requires >=4 drones; using 4 instead of %s", num_drones)
        num_drones = 4

    drones_config = [
        {"id": i + 1, "udp_port": 14551 + i * 10, "role": "chain2"}
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

    duration = float(args.duration)
    exchange_hz = max(0.0, float(getattr(args, "exchange_hz", 50.0)))
    experiments_base = os.path.join(_project_root, "experiments")
    experiment_dir: Optional[str] = None
    if duration > 0:
        if args.experiment_dir:
            experiment_dir = os.path.abspath(args.experiment_dir)
        elif args.run_id:
            experiment_dir = os.path.join(experiments_base, f"exp_{args.run_id}")
        else:
            experiment_dir = os.path.join(
                experiments_base, f"linear_chain2_{time.strftime('%Y%m%d_%H%M%S')}"
            )
        os.makedirs(experiment_dir, exist_ok=True)
        logger.info("Experiment artifacts directory: %s", experiment_dir)

    ids = sorted(int(c.config["id"]) for c in controllers)
    endpoint_left_id = min(ids)
    endpoint_right_id = max(ids)

    metrics_rows: List[Tuple[float, Dict[int, float], Dict[int, float]]] = []
    metrics_lock = threading.Lock()
    metrics_enabled = duration > 0 and experiment_dir is not None

    global START_TIME
    START_TIME = time.time()

    exchange_thread = threading.Thread(
        target=coordinate_exchange_loop,
        args=(
            controllers,
            duration,
            exchange_hz,
            endpoint_left_id,
            endpoint_right_id,
            metrics_lock if metrics_enabled else None,
            metrics_rows if metrics_enabled else None,
        ),
        daemon=not metrics_enabled,
    )
    exchange_thread.start()
    time.sleep(1)
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
                target=internal_chain_anatoliy_loop,
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
        if metrics_enabled:
            exchange_thread.join(timeout=max(30.0, duration * 0.25 + 5.0))
            if exchange_thread.is_alive():
                logger.warning(
                    "Coordinate exchange thread still running; axis metrics may be incomplete."
                )
            if experiment_dir is not None:
                meta: Dict[str, object] = {
                    "scenario": "linear_chain2",
                    "duration_s": duration,
                    "exchange_hz": exchange_hz,
                    "num_drones": len(controllers),
                    "drone_ids": ids,
                    "endpoint_left_id": endpoint_left_id,
                    "endpoint_right_id": endpoint_right_id,
                    "segment_length_m": float(args.segment_length),
                    "r_vis_m": float(args.r_vis),
                    "w_cross_m": float(args.w_cross),
                }
                _finalize_axis_experiment_artifacts(
                    experiment_dir, ids, metrics_rows, meta
                )
        for c in controllers:
            try:
                c.stop()
            except Exception:
                pass


if __name__ == "__main__":
    main()

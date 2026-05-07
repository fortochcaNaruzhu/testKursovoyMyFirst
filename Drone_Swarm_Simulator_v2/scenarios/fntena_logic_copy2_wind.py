"""
Сценарий «fntena_logic_copy2»:
Вертикальная «антенна» над якорем с законом максимально близким к
`Anatoliy/swarm_mavic/grid_antenna` (PointAgent.perform / control_force).
Для SITL горизонталь отделена от вертикального строя: X/Y тянутся к якорной
линии, а соседский закон распределения применяется по Z.

Отличия от "antena_logic copy.py":
- по X/Y/Z считаем sigma_b = v_b - tanh(d_b_plus) + tanh(d_b_minus)
- X/Y удерживаются PID-регулятором в якорной колонне
- Z управляется bang-bang: u_z = -sign(sigma_z) (маппинг прямо в RC PWM)
- выбор соседей как в grid_antenna: ближайший по знаку проекции на каждой оси
  среди видимых по евклидовому радиусу R_vis.

Логи (CSV) расширены: sigma_x/y/z, PWM стиков, скорости vx/vy/vz (NED).
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
from core.logging.csv_logger import (
    CSV_HEADER_ANTENA_TELEMETRY,
    write_metadata,
    write_row_antena_telemetry,
)
from core.mavlink.utils import RC_NEUTRAL

try:
    from visualizer.position_publisher import publish_positions as _publish_positions
except ImportError:
    _publish_positions = None

logger = logging.getLogger(__name__)

START_TIME: float = 0.0
STOP_EVENT = threading.Event()

# Takeoff and anchor target altitude (for the anchor drone only).
TAKEOFF_ALT_M = 1.0

# Common NED frame (align per-drone SITL homes).
_HOME_Y_OFFSET_STEP_M = 2.0

# Control rates.
CONTROL_HZ = 20.0
CONTROL_DT = 1.0 / CONTROL_HZ

# grid_antenna-like parameters
R_VIS_M = 2.5
W_SPACING_M = 0.8
AXIS_EPS_M = 1e-4

# XY lock to anchor column.
XY_OUTPUT_LIMIT = 150.0
XY_DEADBAND_M = 0.015
XY_ROLL_KP = 280.0
XY_ROLL_KI = 12.0
XY_ROLL_KD = 150.0
XY_ROLL_INTEGRAL_LIMIT = 3.0
XY_PITCH_KP = 320.0
XY_PITCH_KI = 12.0
XY_PITCH_KD = 180.0
XY_PITCH_INTEGRAL_LIMIT = 3.0

# Legacy XY bang-bang CLI knobs are accepted for compatibility; PID uses XY_OUTPUT_LIMIT.
XY_PWM_MAX = 110
XY_PWM_MIN_STEP = 18

# Z RC bang-bang mapping (PWM delta around 1500).
# POSHOLD has inertia and delay; pure saturated bang-bang caused a persistent
# throttle "accordion". Agents use a PI layer over sigma_z; anchor uses min-step
# altitude hold.
Z_PWM_MAX = 220
Z_PWM_MIN_STEP = 45
Z_SIGMA_DEADBAND = 0.04
Z_SIGMA_KP = 360.0
Z_SIGMA_KI = 110.0
Z_SIGMA_INTEGRAL_LIMIT = 3.0
BOOTSTRAP_ALT_SEPARATION_SEC = 2.0
BOOTSTRAP_BASE_PWM = 90
BOOTSTRAP_STEP_PWM = 35

# Allow baro/EKF to settle after MAVLink connect before ARM.
POST_CONNECT_SETTLE_SEC = 7.0

INIT_STEPS = [
    {"type": "set_mode", "mode_id": 4},  # GUIDED
    {"type": "sleep", "sec": 1.0},
    {"type": "arm"},
    {"type": "sleep", "sec": 4.0},
    {"type": "takeoff", "alt_m": 5.0},
    {"type": "sleep", "sec": 6.0},
    {"type": "set_mode", "mode_id": 16},  # POSHOLD
    {"type": "sleep", "sec": 1.0},
    {"type": "set_mode", "mode_id": 16},  # POSHOLD retry
    {
        "type": "rc_override",
        "chan1": RC_NEUTRAL,
        "chan2": RC_NEUTRAL,
        "chan3": RC_NEUTRAL,
        "chan4": RC_NEUTRAL,
    },
    {"type": "sleep", "sec": 0.3},
]


def _did_offset_y(did: int) -> float:
    return float(did - 1) * _HOME_Y_OFFSET_STEP_M


def _pos_common(controller: DroneController) -> Dict[str, float]:
    raw = controller.get_my_position()
    did = int(controller.config["id"])
    return {**raw, "y": float(raw.get("y", 0.0)) + _did_offset_y(did)}


def _altitude_m(pos_common: Dict[str, float]) -> float:
    # NED: z positive down, altitude up is -z
    return -float(pos_common.get("z", 0.0))


def _clamp_rc(pwm: int, lo: int = 1100, hi: int = 1900) -> int:
    return max(lo, min(hi, int(pwm)))


def _deadband(x: float, band: float) -> float:
    return 0.0 if abs(x) < band else x


def _pid_xy() -> Tuple[PIDRegulator, PIDRegulator]:
    roll_pid = PIDRegulator(
        kp=XY_ROLL_KP,
        ki=XY_ROLL_KI,
        kd=XY_ROLL_KD,
        integral_limit=XY_ROLL_INTEGRAL_LIMIT,
        output_limit=XY_OUTPUT_LIMIT,
        derivative_alpha=0.65,
    )
    pitch_pid = PIDRegulator(
        kp=XY_PITCH_KP,
        ki=XY_PITCH_KI,
        kd=XY_PITCH_KD,
        integral_limit=XY_PITCH_INTEGRAL_LIMIT,
        output_limit=XY_OUTPUT_LIMIT,
        derivative_alpha=0.65,
    )
    return roll_pid, pitch_pid


def _pid_z_sigma(output_limit: float) -> PIDRegulator:
    return PIDRegulator(
        kp=Z_SIGMA_KP,
        ki=Z_SIGMA_KI,
        kd=0.0,
        integral_limit=Z_SIGMA_INTEGRAL_LIMIT,
        output_limit=float(output_limit),
        derivative_alpha=0.0,
    )


def _xi(g: float) -> float:
    return math.tanh(float(g))


def _ned_velocity(controller: DroneController) -> Tuple[float, float, float]:
    """SIM_STATE velocity in NED (m/s): vz positive down."""
    if controller.velocity_monitor is None:
        return (0.0, 0.0, 0.0)
    try:
        vel = controller.velocity_monitor.get_velocity()
        return (
            float(vel.get("vx", 0.0)),
            float(vel.get("vy", 0.0)),
            float(vel.get("vz", 0.0)),
        )
    except Exception:
        return (0.0, 0.0, 0.0)


def _visible_peers_vectors(
    my_pos: Dict[str, float],
    peer_positions: Dict[int, Dict[str, float]],
    *,
    r_vis: float,
    include_anchor_vec: Optional[Tuple[float, float, float]] = None,
    exclude_ids: Optional[set[int]] = None,
) -> List[Tuple[float, float, float]]:
    """Return list of relative vectors (peer - me) within Euclidean r_vis."""
    mx = float(my_pos.get("x", 0.0))
    my = float(my_pos.get("y", 0.0))
    mz = float(my_pos.get("z", 0.0))
    r = float(r_vis)
    out: List[Tuple[float, float, float]] = []
    if include_anchor_vec is not None:
        ax, ay, az = include_anchor_vec
        if (ax * ax + ay * ay + az * az) ** 0.5 <= r:
            out.append((float(ax), float(ay), float(az)))
    ex = exclude_ids or set()
    for pid, p in peer_positions.items():
        if int(pid) in ex:
            continue
        dx = float(p.get("x", 0.0)) - mx
        dy = float(p.get("y", 0.0)) - my
        dz = float(p.get("z", 0.0)) - mz
        if (dx * dx + dy * dy + dz * dz) ** 0.5 <= r:
            out.append((dx, dy, dz))
    return out


def _nearest_distances_by_axis(
    peers: List[Tuple[float, float, float]],
) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float], Optional[float], Optional[float]]:
    """
    grid_antenna SceneSupervisor.get_mins_ids behavior:
    nearest positive projection and nearest negative (absolute) per axis.
    A peer with ~zero projection is not a neighbor on that axis; otherwise equal-height
    takeoff makes every drone "already have" a z-neighbor and the vertical law never starts.
    """
    x_plus: Optional[float] = None
    y_plus: Optional[float] = None
    z_plus: Optional[float] = None
    x_minus: Optional[float] = None
    y_minus: Optional[float] = None
    z_minus: Optional[float] = None
    for vx, vy, vz in peers:
        if vx > AXIS_EPS_M:
            x_plus = float(vx) if x_plus is None else min(x_plus, float(vx))
        elif vx < -AXIS_EPS_M:
            x_minus = float(-vx) if x_minus is None else min(x_minus, float(-vx))
        if vy > AXIS_EPS_M:
            y_plus = float(vy) if y_plus is None else min(y_plus, float(vy))
        elif vy < -AXIS_EPS_M:
            y_minus = float(-vy) if y_minus is None else min(y_minus, float(-vy))
        if vz > AXIS_EPS_M:
            z_plus = float(vz) if z_plus is None else min(z_plus, float(vz))
        elif vz < -AXIS_EPS_M:
            z_minus = float(-vz) if z_minus is None else min(z_minus, float(-vz))
    return (x_plus, y_plus, z_plus, x_minus, y_minus, z_minus)


def _anchor_dir_from_vec(vec_to_anchor: Tuple[float, float, float]) -> List[float]:
    x, y, z = vec_to_anchor
    return [
        math.copysign(1.0, x) if x != 0 else 0.0,
        math.copysign(1.0, y) if y != 0 else 0.0,
        math.copysign(1.0, z) if z != 0 else 0.0,
        -math.copysign(1.0, x) if x != 0 else 0.0,
        -math.copysign(1.0, y) if y != 0 else 0.0,
        -math.copysign(1.0, z) if z != 0 else 0.0,
    ]


def _distances_grid_antenna_pointagent(
    peers: List[Tuple[float, float, float]],
    *,
    anchor_dir: List[float],
    w: float,
    r_vis: float,
) -> Tuple[float, float, float, float, float, float]:
    """
    Emulate `PointAgent.perform` distances filling (utils.py):
    - if peers present: use nearest distances and fill missing by anchor_dir:
        if anchor_dir[i] <= 0: z dirs -> w, else 0 ; else -> R_vis
    - if no peers: fill by anchor_dir only:
        if item >= 0 => R_vis * item ; else z dirs => -w*item ; else 0
    """
    r = float(r_vis)
    wv = float(w)
    if peers:
        x_p, y_p, z_p, x_m, y_m, z_m = _nearest_distances_by_axis(peers)
        mins = [x_p, y_p, z_p, x_m, y_m, z_m]
        out: List[float] = []
        for i in range(6):
            if mins[i] is None:
                if float(anchor_dir[i]) <= 0:
                    if i in (2, 5):
                        out.append(wv)
                    else:
                        out.append(0.0)
                else:
                    out.append(r)
            else:
                out.append(float(mins[i]))
        return (out[0], out[1], out[2], out[3], out[4], out[5])
    out2: List[float] = []
    for i, item in enumerate(anchor_dir):
        if float(item) >= 0:
            out2.append(r * float(item))
        else:
            if i in (2, 5):
                out2.append(-wv * float(item))
            else:
                out2.append(0.0)
    return (out2[0], out2[1], out2[2], out2[3], out2[4], out2[5])


def _sigma_from_distances(distances: Tuple[float, float, float, float, float, float], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    d_x_plus, d_y_plus, d_z_plus, d_x_minus, d_y_minus, d_z_minus = distances
    vx, vy, vz = v
    sigma_x = float(vx) - _xi(d_x_plus) + _xi(d_x_minus)
    sigma_y = float(vy) - _xi(d_y_plus) + _xi(d_y_minus)
    sigma_z = float(vz) - _xi(d_z_plus) + _xi(d_z_minus)
    return (sigma_x, sigma_y, sigma_z)


def _throttle_delta_from_alt_error(alt_error_m: float, pwm_max: int, pwm_min_step: int) -> int:
    """Positive altitude error means climb, so throttle must go above neutral."""
    err = float(alt_error_m)
    if abs(err) < 1e-9:
        return 0
    mag = min(1.0, abs(err) / 2.0)
    delta = int(round(float(pwm_max) * mag))
    if delta < int(pwm_min_step):
        delta = int(pwm_min_step)
    return int(round(math.copysign(delta, err)))


def initialize_drone_parallel(
    controller: DroneController,
    init_barrier: threading.Barrier,
    *,
    heartbeat_timeout_s: Optional[float],
    position_timeout_s: float = 10.0,
    barrier_timeout_sec: float = 60.0,
) -> None:
    try:
        did = controller.config.get("id")
        stagger_s = 0.35 * float(max(0, int(did) - 1))
        if stagger_s > 0:
            time.sleep(stagger_s)
        if heartbeat_timeout_s is not None:
            controller.connect_with_heartbeat_timeout(float(heartbeat_timeout_s))
        else:
            controller.connect()
        time.sleep(float(POST_CONNECT_SETTLE_SEC))
        if not controller.initialize(list(INIT_STEPS)):
            raise TimeoutError(f"Drone {did}: MAVLink init sequence timed out.")
        controller.start_rc_keepalive()
        t0 = time.time()
        while time.time() - t0 < float(position_timeout_s):
            if controller.get_position() is not None:
                break
            time.sleep(0.1)
        else:
            raise TimeoutError("No SIM_STATE position after initialization")
        init_barrier.wait(timeout=float(barrier_timeout_sec))
    except Exception:
        try:
            init_barrier.abort()
        except Exception:
            pass
        logger.exception("[fntena_logic_copy2] Drone init failed (id=%s)", controller.config.get("id"))


def _stop_all(controllers: List[DroneController]) -> None:
    for c in controllers:
        try:
            c.stop()
        except Exception:
            pass


def _bootstrap_altitude_separation(controllers: List[DroneController], *, anchor_id: int) -> None:
    """Break equal-z symmetry before the local nearest-neighbor z law starts."""
    t_end = time.time() + float(BOOTSTRAP_ALT_SEPARATION_SEC)
    while time.time() < t_end:
        for c in controllers:
            did = int(c.config["id"])
            if did == int(anchor_id) or c.worker is None:
                continue
            rank = max(1, did - int(anchor_id))
            boost_pwm = int(BOOTSTRAP_BASE_PWM + BOOTSTRAP_STEP_PWM * rank)
            c.worker.send_rc_override(
                RC_NEUTRAL,
                RC_NEUTRAL,
                _clamp_rc(RC_NEUTRAL + boost_pwm),
                RC_NEUTRAL,
                controller=c,
            )
        time.sleep(0.1)


def _control_loop(
    controller: DroneController,
    *,
    anchor_id: int,
    duration_s: float,
    r_vis_m: float,
    w_spacing_m: float,
    xy_pwm_max: int,
    xy_pwm_min_step: int,
    z_pwm_max: int,
    z_pwm_min_step: int,
) -> None:
    global START_TIME
    did = int(controller.config["id"])
    if controller.worker is None:
        return

    roll_pid, pitch_pid = _pid_xy()
    z_sigma_pid = _pid_z_sigma(float(z_pwm_max))

    while True:
        if duration_s > 0 and (time.time() - START_TIME) >= duration_s:
            return

        my_pos = _pos_common(controller)
        others = controller.get_other_drones_positions()
        anchor_pos = my_pos if did == int(anchor_id) else others.get(int(anchor_id))
        vx, vy, vz = _ned_velocity(controller)

        if anchor_pos is None:
            controller._antena_telemetry = {
                "sigma_x": 0.0,
                "sigma_y": 0.0,
                "sigma_z": 0.0,
                "rc_roll": float(RC_NEUTRAL),
                "rc_pitch": float(RC_NEUTRAL),
                "rc_throttle": float(RC_NEUTRAL),
                "rc_yaw": float(RC_NEUTRAL),
                "vx": vx,
                "vy": vy,
                "vz": vz,
            }
            controller.worker.send_rc_override(
                RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, controller=controller
            )
            time.sleep(CONTROL_DT)
            continue

        # Build peers vectors: include "attraction point" = anchor pose.
        mx = float(my_pos.get("x", 0.0))
        my = float(my_pos.get("y", 0.0))
        mz = float(my_pos.get("z", 0.0))
        ax = float(anchor_pos.get("x", 0.0))
        ay = float(anchor_pos.get("y", 0.0))
        az = float(anchor_pos.get("z", 0.0))
        vec_to_anchor = (ax - mx, ay - my, az - mz)

        peers = _visible_peers_vectors(
            my_pos,
            others,
            r_vis=float(r_vis_m),
            include_anchor_vec=vec_to_anchor if did != int(anchor_id) else None,
            exclude_ids={int(anchor_id)} if did != int(anchor_id) else set(),
        )
        anchor_dir = _anchor_dir_from_vec(vec_to_anchor)
        distances = _distances_grid_antenna_pointagent(
            peers,
            anchor_dir=anchor_dir,
            w=float(w_spacing_m),
            r_vis=float(r_vis_m),
        )

        _, _, sigma_z = _sigma_from_distances(distances, (vx, vy, vz))
        sigma_x = _deadband(ax - mx, XY_DEADBAND_M)
        sigma_y = _deadband(ay - my, XY_DEADBAND_M)

        # Convention aligned with previous scenario:
        # pitch affects +x, roll affects +y; throttle affects -z (altitude up is -z).
        pitch_out = pitch_pid.update(sigma_x, dt=CONTROL_DT)
        roll_out = roll_pid.update(sigma_y, dt=CONTROL_DT)

        # For throttle: sigma_z is grid_antenna-like and includes NED vz damping.
        # A PI layer converts persistent sigma error into enough RC authority for POSHOLD.
        sigma_z_ctrl = _deadband(sigma_z, Z_SIGMA_DEADBAND)
        thr_delta = int(round(z_sigma_pid.update(sigma_z_ctrl, dt=CONTROL_DT)))

        pitch = _clamp_rc(RC_NEUTRAL - int(pitch_out))
        roll = _clamp_rc(RC_NEUTRAL + int(roll_out))
        throttle = _clamp_rc(RC_NEUTRAL + int(thr_delta))

        # Anchor drone: keep fixed horizontal position and altitude target to act as pacemaker.
        if did == int(anchor_id):
            # Keep anchor at its own initial point: by definition its "attraction" is itself.
            # We set sigmas from the same computation but neutralize stick to avoid drift.
            roll = RC_NEUTRAL
            pitch = RC_NEUTRAL
            # Keep at TAKEOFF_ALT_M.
            alt_err = float(TAKEOFF_ALT_M) - _altitude_m(my_pos)
            # Reuse sigma_z channel for logging: "error-like" scalar.
            sigma_z = alt_err
            thr_delta = _throttle_delta_from_alt_error(sigma_z, int(z_pwm_max), int(z_pwm_min_step))
            throttle = _clamp_rc(RC_NEUTRAL + int(thr_delta))

        controller._antena_telemetry = {
            "sigma_x": float(sigma_x),
            "sigma_y": float(sigma_y),
            "sigma_z": float(sigma_z),
            "rc_roll": float(roll),
            "rc_pitch": float(pitch),
            "rc_throttle": float(throttle),
            "rc_yaw": float(RC_NEUTRAL),
            "vx": vx,
            "vy": vy,
            "vz": vz,
        }

        controller.worker.send_rc_override(roll, pitch, throttle, RC_NEUTRAL, controller=controller)
        time.sleep(CONTROL_DT)


def main() -> None:
    import argparse

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    parser = argparse.ArgumentParser(description="fntena_logic_copy2: PID XY anchor-column lock with grid_antenna-like Z control in SITL")
    parser.add_argument("--drones", type=int, default=4)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--heartbeat-timeout", type=float, default=12.0)
    parser.add_argument("--exchange-hz", type=float, default=50.0, help="Accepted for launcher compatibility; ignored.")
    parser.add_argument("--anchor-id", type=int, default=1)

    parser.add_argument("--r-vis", type=float, default=R_VIS_M)
    parser.add_argument("--w", type=float, default=W_SPACING_M)
    parser.add_argument("--xy-pwm-max", type=int, default=XY_PWM_MAX)
    parser.add_argument("--xy-pwm-min-step", type=int, default=XY_PWM_MIN_STEP)
    parser.add_argument("--z-pwm-max", type=int, default=Z_PWM_MAX)
    parser.add_argument("--z-pwm-min-step", type=int, default=Z_PWM_MIN_STEP)

    parser.add_argument("--experiment-dir", type=str, default=None)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--log-hz", type=float, default=20.0)
    parser.add_argument(
        "--log-mode",
        type=str,
        default="timer",
        choices=["timer", "mavlink"],
        help="CSV logging mode. 'timer' logs at --log-hz. 'mavlink' logs each new SIM_STATE position sample.",
    )
    args = parser.parse_args()

    if float(args.r_vis) <= 0:
        parser.error("--r-vis must be positive.")
    if float(args.w) <= 0:
        parser.error("--w must be positive.")
    if float(args.w) >= float(args.r_vis):
        parser.error("--w must be less than --r-vis so the requested spacing is reachable within the visibility radius.")
    if int(args.z_pwm_max) <= 0:
        parser.error("--z-pwm-max must be positive.")
    if int(args.z_pwm_min_step) < 0:
        parser.error("--z-pwm-min-step must be non-negative.")

    num_drones = max(1, int(args.drones))
    anchor_id = int(args.anchor_id)
    if anchor_id < 1 or anchor_id > num_drones:
        anchor_id = 1

    drones_config: List[Dict[str, object]] = [
        {"id": i + 1, "udp_port": 14551 + i * 10, "role": "fntena_logic_copy2"} for i in range(num_drones)
    ]
    controllers: List[DroneController] = [DroneController(cfg, logging_enabled=False) for cfg in drones_config]

    init_barrier = threading.Barrier(len(controllers) + 1)
    for c in controllers:
        threading.Thread(
            target=initialize_drone_parallel,
            args=(c, init_barrier),
            kwargs={"heartbeat_timeout_s": float(args.heartbeat_timeout), "position_timeout_s": 10.0},
            daemon=False,
        ).start()

    try:
        init_barrier.wait(timeout=75.0)
    except threading.BrokenBarrierError:
        logger.error("[fntena_logic_copy2] Init barrier broken; stopping.")
        _stop_all(controllers)
        return

    time.sleep(2.0)
    _bootstrap_altitude_separation(controllers, anchor_id=anchor_id)

    # Experiment logging directory (RViz2 replay format).
    experiment_dir = args.experiment_dir
    if experiment_dir is None:
        if args.run_id:
            experiment_dir = os.path.join(_project_root, "experiments", f"exp_{args.run_id}")
        else:
            experiment_dir = os.path.join(_project_root, "experiments", time.strftime("fntena_logic_copy2_%Y-%m-%d_%H-%M-%S"))
    experiment_dir = os.path.abspath(str(experiment_dir))
    os.makedirs(experiment_dir, exist_ok=True)

    experiment_log_files: Dict[int, object] = {}
    for cfg in drones_config:
        did = int(cfg["id"])
        path = os.path.join(experiment_dir, f"drone_{did}.csv")
        f = open(path, "w", encoding="utf-8")
        f.write(CSV_HEADER_ANTENA_TELEMETRY + "\n")
        f.flush()
        experiment_log_files[did] = f

    write_metadata(
        experiment_dir,
        float(args.duration),
        0.2,
        num_drones,
        "fntena_logic_copy2",
        extra={
            "r_vis_m": float(args.r_vis),
            "w_spacing_m": float(args.w),
            "anchor_id": anchor_id,
            "log_hz": float(args.log_hz),
            "log_mode": str(args.log_mode),
            "xy_output_limit": float(XY_OUTPUT_LIMIT),
            "xy_deadband_m": float(XY_DEADBAND_M),
            "xy_roll_kp": float(XY_ROLL_KP),
            "xy_roll_ki": float(XY_ROLL_KI),
            "xy_roll_kd": float(XY_ROLL_KD),
            "xy_roll_integral_limit": float(XY_ROLL_INTEGRAL_LIMIT),
            "xy_pitch_kp": float(XY_PITCH_KP),
            "xy_pitch_ki": float(XY_PITCH_KI),
            "xy_pitch_kd": float(XY_PITCH_KD),
            "xy_pitch_integral_limit": float(XY_PITCH_INTEGRAL_LIMIT),
            "z_pwm_max": int(args.z_pwm_max),
            "z_pwm_min_step": int(args.z_pwm_min_step),
            "z_sigma_deadband": float(Z_SIGMA_DEADBAND),
            "z_sigma_kp": float(Z_SIGMA_KP),
            "z_sigma_ki": float(Z_SIGMA_KI),
            "z_sigma_integral_limit": float(Z_SIGMA_INTEGRAL_LIMIT),
            "bootstrap_alt_separation_sec": float(BOOTSTRAP_ALT_SEPARATION_SEC),
            "bootstrap_base_pwm": int(BOOTSTRAP_BASE_PWM),
            "bootstrap_step_pwm": int(BOOTSTRAP_STEP_PWM),
        },
    )
    logger.info("[fntena_logic_copy2] Logging RViz replay CSVs to: %s", experiment_dir)

    def exchange_loop() -> None:
        last_pub = 0.0
        pub_period = 1.0 / 20.0
        last_log_time = 0.0
        log_hz = float(getattr(args, "log_hz", 20.0))
        log_period = (1.0 / log_hz) if log_hz and log_hz > 0 else 0.0
        log_mode = str(getattr(args, "log_mode", "timer"))
        last_pos_seq = {int(c.config["id"]): 0 for c in controllers}
        while True:
            if STOP_EVENT.is_set():
                return
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
            if log_mode == "mavlink":
                for c in controllers:
                    did = int(c.config["id"])
                    w = getattr(c, "worker", None)
                    if w is None:
                        continue
                    res = w.wait_for_new_position(last_pos_seq.get(did, 0), timeout_s=0.0)
                    if res is None:
                        continue
                    seq, _pos_raw, sitl_boot_s = res
                    last_pos_seq[did] = seq
                    t_rel = time.time() - START_TIME if START_TIME > 0 else 0.0
                    p = pos_common.get(did) or {"x": 0.0, "y": 0.0, "z": 0.0}
                    att = {"rx": 0.0, "ry": 0.0, "rz": 0.0}
                    if c.coords_monitor is not None:
                        try:
                            att = c.coords_monitor.get_attitude()
                        except Exception:
                            att = att
                    try:
                        tm = getattr(c, "_antena_telemetry", {}) or {}
                        write_row_antena_telemetry(
                            experiment_log_files[did],
                            did,
                            float(t_rel),
                            float(p.get("x", 0.0)),
                            float(p.get("y", 0.0)),
                            float(p.get("z", 0.0)),
                            float(att.get("rx", 0.0)),
                            float(att.get("ry", 0.0)),
                            float(att.get("rz", 0.0)),
                            0,
                            sitl_time_boot_s=sitl_boot_s,
                            sigma_x=float(tm.get("sigma_x", 0.0)),
                            sigma_y=float(tm.get("sigma_y", 0.0)),
                            sigma_z=float(tm.get("sigma_z", 0.0)),
                            rc_roll=int(tm.get("rc_roll", RC_NEUTRAL)),
                            rc_pitch=int(tm.get("rc_pitch", RC_NEUTRAL)),
                            rc_throttle=int(tm.get("rc_throttle", RC_NEUTRAL)),
                            rc_yaw=int(tm.get("rc_yaw", RC_NEUTRAL)),
                            vx=float(tm.get("vx", 0.0)),
                            vy=float(tm.get("vy", 0.0)),
                            vz=float(tm.get("vz", 0.0)),
                        )
                    except Exception:
                        pass
            else:
                if log_period <= 0.0 or (now - last_log_time) >= log_period:
                    t_rel = now - START_TIME if START_TIME > 0 else 0.0
                    for c in controllers:
                        did = int(c.config["id"])
                        p = pos_common.get(did) or {"x": 0.0, "y": 0.0, "z": 0.0}
                        att = {"rx": 0.0, "ry": 0.0, "rz": 0.0}
                        if c.coords_monitor is not None:
                            try:
                                att = c.coords_monitor.get_attitude()
                            except Exception:
                                att = att
                        try:
                            tm = getattr(c, "_antena_telemetry", {}) or {}
                            write_row_antena_telemetry(
                                experiment_log_files[did],
                                did,
                                float(t_rel),
                                float(p.get("x", 0.0)),
                                float(p.get("y", 0.0)),
                                float(p.get("z", 0.0)),
                                float(att.get("rx", 0.0)),
                                float(att.get("ry", 0.0)),
                                float(att.get("rz", 0.0)),
                                0,
                                sitl_time_boot_s=None,
                                sigma_x=float(tm.get("sigma_x", 0.0)),
                                sigma_y=float(tm.get("sigma_y", 0.0)),
                                sigma_z=float(tm.get("sigma_z", 0.0)),
                                rc_roll=int(tm.get("rc_roll", RC_NEUTRAL)),
                                rc_pitch=int(tm.get("rc_pitch", RC_NEUTRAL)),
                                rc_throttle=int(tm.get("rc_throttle", RC_NEUTRAL)),
                                rc_yaw=int(tm.get("rc_yaw", RC_NEUTRAL)),
                                vx=float(tm.get("vx", 0.0)),
                                vy=float(tm.get("vy", 0.0)),
                                vz=float(tm.get("vz", 0.0)),
                            )
                        except Exception:
                            pass
                    last_log_time = now
            if _publish_positions is not None and (now - last_pub) >= pub_period:
                try:
                    _publish_positions(pos_common, rates={"exchange_hz": 1.0 / 0.02})
                except Exception:
                    pass
                last_pub = now
            time.sleep(0.02)

    threading.Thread(target=exchange_loop, daemon=True).start()

    global START_TIME
    START_TIME = time.time()

    duration_s = float(args.duration)
    threads: List[threading.Thread] = []
    for c in controllers:
        t = threading.Thread(
            target=_control_loop,
            args=(c,),
            kwargs={
                "anchor_id": anchor_id,
                "duration_s": duration_s,
                "r_vis_m": float(args.r_vis),
                "w_spacing_m": float(args.w),
                "xy_pwm_max": int(args.xy_pwm_max),
                "xy_pwm_min_step": int(args.xy_pwm_min_step),
                "z_pwm_max": int(args.z_pwm_max),
                "z_pwm_min_step": int(args.z_pwm_min_step),
            },
            daemon=False,
        )
        t.start()
        threads.append(t)

    try:
        if duration_s > 0:
            for t in threads:
                t.join()
        else:
            threading.Event().wait()
    except KeyboardInterrupt:
        logger.info("[fntena_logic_copy2] KeyboardInterrupt: stopping.")
    finally:
        STOP_EVENT.set()
        _stop_all(controllers)
        for _did, f in experiment_log_files.items():
            try:
                f.close()
            except Exception:
                pass
        logger.info("[fntena_logic_copy2] Logs closed. Replay:")
        logger.info("  source /opt/ros/jazzy/setup.bash")
        logger.info("  cd %s", _project_root)
        logger.info("  python3 replay/replay_rviz2.py --experiment %s --rate 1.0 --rviz", experiment_dir)


if __name__ == "__main__":
    main()


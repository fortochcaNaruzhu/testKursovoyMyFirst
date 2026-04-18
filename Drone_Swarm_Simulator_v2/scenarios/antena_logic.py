"""
Сценарий «antena_logic»:
4 дрона взлетают и само-выстраиваются в «вертикальную антенну» над якорем
по локальному правилу ближайших соседей (без привязки к ID в логике высотного строя).

Идея по мотивам KA008 (равномерная решётка по скалярному состоянию) и проекта
Anatoliy/swarm_mavic/grid_antenna: каждый агент использует только ближайшего
видимого соседа сверху/снизу по высоте и/или якорь как пейсмейкер.

Управление через RC override:
- roll/pitch: мягкий PID на удержание оси (x,y) якоря
- throttle: bang-bang / насыщенная команда по знаку sigma (altitude self-distribution)
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
from core.logging.csv_logger import CSV_HEADER, write_metadata, write_row
from core.mavlink.utils import RC_NEUTRAL

try:
    from visualizer.position_publisher import publish_positions as _publish_positions
except ImportError:
    _publish_positions = None

logger = logging.getLogger(__name__)

START_TIME: float = 0.0
STOP_EVENT = threading.Event()

TAKEOFF_ALT_M = 1.0

# Common NED frame (align per-drone SITL homes)
_HOME_Y_OFFSET_STEP_M = 2.0

# Control rates
CONTROL_HZ = 20.0
CONTROL_DT = 1.0 / CONTROL_HZ

# XY lock to anchor line
XY_OUTPUT_LIMIT = 110.0
XY_DEADBAND_M = 0.06

# Altitude self-distribution (scalar state = altitude, m)
W_SPACING_M = 0.8
R_VIS_ALT_M = 2.5
# Для высотного закона лучше НЕ фильтровать по XY: статья опирается на скалярное состояние,
# а у нас на старте дроны разнесены по Y (SITL home). Иначе соседи "не видны" и строй не формируется.
XY_VIS_M = 50.0

# Bang-bang mapping to throttle PWM
Z_PWM_MAX = 95
Z_PWM_MIN_STEP = 18

ANCHOR_Z_PID_OUTPUT_LIMIT = 220.0

INIT_STEPS = [
    {"type": "set_mode", "mode_id": 4},  # GUIDED
    {"type": "sleep", "sec": 0.5},
    {"type": "arm"},
    {"type": "sleep", "sec": 2.0},
    {"type": "takeoff"},
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


def _xi(g: float) -> float:
    return math.tanh(float(g))


def _nearest_above_below_alt(
    my_pos: Dict[str, float],
    peer_positions: Dict[int, Dict[str, float]],
    *,
    r_vis_alt_m: float,
    xy_vis_m: float,
) -> Tuple[Optional[float], Optional[float]]:
    """
    Return (d_plus, d_minus) in meters along altitude axis:
    - d_plus: distance to nearest peer above (higher altitude)
    - d_minus: distance to nearest peer below (lower altitude)
    Only peers within |Δalt| < r_vis_alt_m and within xy_vis_m in horizontal plane are considered.
    """
    my_alt = _altitude_m(my_pos)
    mx = float(my_pos.get("x", 0.0))
    my = float(my_pos.get("y", 0.0))
    d_plus: Optional[float] = None
    d_minus: Optional[float] = None
    for _pid, p in peer_positions.items():
        px = float(p.get("x", 0.0))
        py = float(p.get("y", 0.0))
        dx = px - mx
        dy = py - my
        # xy_vis_m can be set very large to effectively disable gating.
        if float(xy_vis_m) > 0 and (dx * dx + dy * dy) ** 0.5 > float(xy_vis_m):
            continue
        da = _altitude_m(p) - my_alt
        if abs(da) >= float(r_vis_alt_m) or abs(da) < 1e-6:
            continue
        if da > 0:
            d = float(da)
            d_plus = d if d_plus is None else min(d_plus, d)
        else:
            d = float(-da)
            d_minus = d if d_minus is None else min(d_minus, d)
    return d_plus, d_minus


def _sigma_altitude(
    d_plus: Optional[float],
    d_minus: Optional[float],
    *,
    w: float,
    r_vis: float,
    v_alt: float,
) -> float:
    """
    One-dimensional sigma from KA008/grid_antenna style:
      sigma = v - xi(d_plus_eff) + xi(d_minus_eff)
    Where missing neighbor distances are replaced by:
      d_plus_eff = r_vis (no peer above)
      d_minus_eff = w     (no peer below, i.e., toward the anchor side)
    """
    dp = float(r_vis) if d_plus is None else float(d_plus)
    dm = float(w) if d_minus is None else float(d_minus)
    return float(v_alt) - _xi(dp) + _xi(dm)


def _pwm_from_sigma(sigma: float, pwm_max: int, pwm_min_step: int) -> int:
    """
    Convert sigma to throttle delta in PWM around 1500.
    Uses sign(sigma) (bang-bang) with mild scaling by |sigma|.
    Positive return means "go up" (increase altitude) => throttle above neutral.
    """
    s = float(sigma)
    if abs(s) < 1e-6:
        return 0
    direction = -1.0 if s > 0 else 1.0  # u = -sign(sigma)
    mag = min(1.0, abs(s) / 2.0)  # sigma typically ~[-2,2]
    delta = int(round(float(pwm_max) * mag))
    if delta < int(pwm_min_step):
        delta = int(pwm_min_step)
    return int(round(direction * delta))


def _pid_xy() -> Tuple[PIDRegulator, PIDRegulator]:
    roll_pid = PIDRegulator(
        kp=160.0,
        ki=0.0,
        kd=150.0,
        integral_limit=60.0,
        output_limit=XY_OUTPUT_LIMIT,
        derivative_alpha=0.65,
    )
    pitch_pid = PIDRegulator(
        kp=210.0,
        ki=0.0,
        kd=180.0,
        integral_limit=60.0,
        output_limit=XY_OUTPUT_LIMIT,
        derivative_alpha=0.65,
    )
    return roll_pid, pitch_pid


def _pid_anchor_z() -> PIDRegulator:
    """Anchor altitude hold PID (maps altitude error to throttle PWM delta)."""
    return PIDRegulator(
        kp=300.0,
        ki=0.0,
        kd=220.0,
        integral_limit=80.0,
        output_limit=float(ANCHOR_Z_PID_OUTPUT_LIMIT),
        derivative_alpha=0.7,
    )


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
        if heartbeat_timeout_s is not None:
            logger.info(
                "[antena_logic] Drone %s: connecting with heartbeat timeout %.1fs",
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
        logger.exception("[antena_logic] Drone init failed (id=%s)", controller.config.get("id"))


def _stop_all(controllers: List[DroneController]) -> None:
    for c in controllers:
        try:
            c.stop()
        except Exception:
            pass


def _control_loop(
    controller: DroneController,
    *,
    anchor_id: int,
    duration_s: float,
    w_spacing_m: float,
    r_vis_alt_m: float,
    xy_vis_m: float,
    z_pwm_max: int,
    z_pwm_min_step: int,
) -> None:
    global START_TIME
    did = int(controller.config["id"])
    if controller.worker is None:
        return

    roll_pid, pitch_pid = _pid_xy()
    anchor_z_pid = _pid_anchor_z() if did == int(anchor_id) else None

    while True:
        if duration_s > 0 and (time.time() - START_TIME) >= duration_s:
            return

        my_pos = _pos_common(controller)

        others = controller.get_other_drones_positions()
        anchor_pos = others.get(int(anchor_id))
        if anchor_pos is None:
            controller.worker.send_rc_override(
                RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, RC_NEUTRAL, controller=controller
            )
            time.sleep(CONTROL_DT)
            continue

        # --- XY lock to anchor column ---
        err_x = _deadband(float(anchor_pos.get("x", 0.0)) - float(my_pos.get("x", 0.0)), XY_DEADBAND_M)
        err_y = _deadband(float(anchor_pos.get("y", 0.0)) - float(my_pos.get("y", 0.0)), XY_DEADBAND_M)
        pitch_out = pitch_pid.update(err_x, dt=CONTROL_DT)
        roll_out = roll_pid.update(err_y, dt=CONTROL_DT)
        pitch = _clamp_rc(RC_NEUTRAL - int(pitch_out))
        roll = _clamp_rc(RC_NEUTRAL + int(roll_out))

        # --- Anchor: hold altitude at TAKEOFF_ALT_M (do not participate in distribution) ---
        if did == int(anchor_id) and anchor_z_pid is not None:
            alt_me = _altitude_m(my_pos)
            alt_err = _deadband(float(TAKEOFF_ALT_M) - float(alt_me), 0.05)
            # Positive error => need go up => increase throttle above neutral.
            thr_out = anchor_z_pid.update(alt_err, dt=CONTROL_DT)
            throttle = _clamp_rc(RC_NEUTRAL + int(thr_out))
            controller.worker.send_rc_override(
                roll, pitch, throttle, RC_NEUTRAL, controller=controller
            )
            time.sleep(CONTROL_DT)
            continue

        # --- Altitude self-distribution (anchor acts as virtual peer below the lattice) ---
        peers_for_alt: Dict[int, Dict[str, float]] = dict(others)
        peers_for_alt[int(anchor_id)] = anchor_pos

        d_plus, d_minus = _nearest_above_below_alt(
            my_pos,
            peers_for_alt,
            r_vis_alt_m=float(r_vis_alt_m),
            xy_vis_m=float(xy_vis_m),
        )

        # Altitude speed from LOCAL_POSITION_NED: vz is DOWN positive => v_alt = -vz.
        v_alt = 0.0
        if controller.velocity_monitor is not None:
            try:
                vel = controller.velocity_monitor.get_velocity()
                v_alt = -float(vel.get("vz", 0.0))
            except Exception:
                v_alt = 0.0

        sigma = _sigma_altitude(
            d_plus,
            d_minus,
            w=float(w_spacing_m),
            r_vis=float(r_vis_alt_m),
            v_alt=float(v_alt),
        )
        thr_delta = _pwm_from_sigma(sigma, int(z_pwm_max), int(z_pwm_min_step))
        throttle = _clamp_rc(RC_NEUTRAL + int(thr_delta))

        controller.worker.send_rc_override(roll, pitch, throttle, RC_NEUTRAL, controller=controller)
        time.sleep(CONTROL_DT)


def main() -> None:
    import argparse

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    parser = argparse.ArgumentParser(description="antena_logic: nearest-neighbor vertical antenna (rc_override)")
    parser.add_argument("--drones", type=int, default=4, help="Launcher compatibility; coerced to 4")
    parser.add_argument("--duration", type=float, default=0.0, help="Run duration (s); 0 = infinite")
    parser.add_argument("--heartbeat-timeout", type=float, default=12.0, help="Heartbeat timeout (s)")
    parser.add_argument("--exchange-hz", type=float, default=50.0, help="Accepted for launcher compatibility; ignored.")
    parser.add_argument("--anchor-id", type=int, default=1, help="Anchor drone id (pacemaker, default 1)")

    parser.add_argument("--w", type=float, default=W_SPACING_M, help="Desired vertical spacing (m)")
    parser.add_argument("--r-vis", type=float, default=R_VIS_ALT_M, help="Visibility distance along altitude (m)")
    parser.add_argument("--xy-vis", type=float, default=XY_VIS_M, help="Horizontal gating distance to consider a peer (m)")
    parser.add_argument("--z-pwm-max", type=int, default=Z_PWM_MAX, help="Max throttle PWM delta from neutral for sigma")
    parser.add_argument("--z-pwm-min-step", type=int, default=Z_PWM_MIN_STEP, help="Min PWM step when sigma != 0")

    parser.add_argument(
        "--experiment-dir",
        type=str,
        default=None,
        help="Directory for RViz replay logs (metadata.json + drone_*.csv). If omitted, uses experiments/...",
    )
    parser.add_argument(
        "--run-id",
        type=str,
        default=None,
        help="If --experiment-dir omitted: use experiments/exp_<run-id> (launcher compatibility).",
    )
    parser.add_argument(
        "--log-hz",
        type=float,
        default=20.0,
        help="Log write rate (Hz). 0 = write every exchange tick (50 Hz).",
    )

    args = parser.parse_args()

    num_drones = int(args.drones)
    if num_drones != 4:
        logger.info("[antena_logic] Coercing --drones from %d to 4.", num_drones)
        num_drones = 4

    anchor_id = int(args.anchor_id)
    if anchor_id < 1 or anchor_id > num_drones:
        anchor_id = 1

    drones_config: List[Dict[str, object]] = [
        {"id": i + 1, "udp_port": 14551 + i * 10, "role": "antena_logic"} for i in range(num_drones)
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
        logger.error("[antena_logic] Init barrier broken; stopping.")
        _stop_all(controllers)
        return

    time.sleep(2.0)

    # Bootstrap: create a small altitude separation so agents "see" anchor below
    # and the nearest-neighbor altitude law has something to work with.
    # Without this, all drones can start at the same altitude (~1m) and remain symmetric.
    def _bootstrap_altitude_separation() -> None:
        # Break altitude symmetry: apply different small boosts per drone (derived from id)
        # so that nearest-neighbor distances along altitude become non-zero.
        boost_sec = 2.0
        base_pwm = 90
        step_pwm = 35
        t_end = time.time() + boost_sec
        while time.time() < t_end:
            for c in controllers:
                did = int(c.config["id"])
                if did == int(anchor_id) or c.worker is None:
                    continue
                rank = max(1, did - int(anchor_id))  # 1.. for non-anchor
                boost_pwm = int(base_pwm + step_pwm * rank)
                c.worker.send_rc_override(
                    RC_NEUTRAL,
                    RC_NEUTRAL,
                    _clamp_rc(RC_NEUTRAL + boost_pwm),
                    RC_NEUTRAL,
                    controller=c,
                )
            time.sleep(0.1)

    _bootstrap_altitude_separation()

    # Experiment logging directory (RViz2 replay format: metadata.json + drone_*.csv).
    experiment_dir = args.experiment_dir
    if experiment_dir is None:
        if args.run_id:
            experiment_dir = os.path.join(_project_root, "experiments", f"exp_{args.run_id}")
        else:
            experiment_dir = os.path.join(
                _project_root, "experiments", time.strftime("antena_logic_%Y-%m-%d_%H-%M-%S")
            )
    experiment_dir = os.path.abspath(str(experiment_dir))
    os.makedirs(experiment_dir, exist_ok=True)

    experiment_log_files: Dict[int, object] = {}
    for cfg in drones_config:
        did = int(cfg["id"])
        path = os.path.join(experiment_dir, f"drone_{did}.csv")
        f = open(path, "w", encoding="utf-8")
        f.write(CSV_HEADER + "\n")
        f.flush()
        experiment_log_files[did] = f

    write_metadata(
        experiment_dir,
        float(args.duration),
        0.2,
        num_drones,
        "antena_logic",
        extra={
            "w_spacing_m": float(args.w),
            "r_vis_alt_m": float(args.r_vis),
            "xy_vis_m": float(args.xy_vis),
            "anchor_id": anchor_id,
            "log_hz": float(args.log_hz),
        },
    )
    logger.info("[antena_logic] Logging RViz replay CSVs to: %s", experiment_dir)

    # Position exchange + visualizer publishing (common frame).
    def exchange_loop() -> None:
        last_pub = 0.0
        pub_period = 1.0 / 20.0
        last_log_time = 0.0
        log_hz = float(getattr(args, "log_hz", 20.0))
        log_period = (1.0 / log_hz) if log_hz and log_hz > 0 else 0.0
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
            # Write per-drone CSV logs for RViz replay (NED, common frame).
            now = time.time()
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
                        write_row(
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
                "w_spacing_m": float(args.w),
                "r_vis_alt_m": float(args.r_vis),
                "xy_vis_m": float(args.xy_vis),
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
        logger.info("[antena_logic] KeyboardInterrupt: stopping.")
    finally:
        STOP_EVENT.set()
        _stop_all(controllers)
        for _did, f in experiment_log_files.items():
            try:
                f.close()
            except Exception:
                pass
        logger.info("[antena_logic] Logs closed. Replay:")
        logger.info("  source /opt/ros/jazzy/setup.bash")
        logger.info("  cd %s", _project_root)
        logger.info("  python3 replay/replay_rviz2.py --experiment %s --rate 1.0 --rviz", experiment_dir)


if __name__ == "__main__":
    main()


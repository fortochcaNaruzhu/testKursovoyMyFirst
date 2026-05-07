#!/usr/bin/env python3
"""
ROS 2 node: 3D replay of drone swarm experiments in RViz 2.

Loads the same experiment format as replay_rviz.py (metadata.json + drone_*.csv).
Publishes geometry_msgs/PoseStamped on /swarm/drone_<id>/pose (ENU, converted from NED).
Also publishes visualization_msgs/MarkerArray on /swarm/markers — one RViz2 display
(Add → By topic) shows every drone without wiring each /pose separately.

Dependencies: ROS 2 (e.g. Humble/Jazzy), rclpy, geometry_msgs, std_msgs, visualization_msgs.
Optional: same interactive topics as ROS1 (/replay/play, pause, seek, speed).

Usage (from project root, after: source /opt/ros/jazzy/setup.bash):
  python replay/replay_rviz2.py --experiment experiments/exp_1 --rate 1.0
  python replay/replay_rviz2.py --experiment experiments/exp_1 --rviz   # + RViz2 автоматически
  # По умолчанию маркеры — visualizer/models/iris.dae; сферы: --marker-spheres
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import select
import shutil
import signal
import subprocess
import sys
import threading
import time
import tty
import termios
from typing import Any, Dict, List, Optional, Tuple

_script_dir = os.path.dirname(os.path.abspath(__file__))
_replay_parent = os.path.dirname(_script_dir)
if _replay_parent not in sys.path:
    sys.path.insert(0, _replay_parent)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import ColorRGBA, Empty, Float32, Float64, String
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError as e:
    print(
        "replay_rviz2.py requires ROS 2 and rclpy. Example:\n"
        "  source /opt/ros/humble/setup.bash\n"
        "Error: " + str(e),
        file=sys.stderr,
    )
    sys.exit(1)

from replay.csv_loader import load_experiment, iter_steps
from replay.playback_controller import PlaybackState, SPEED_MAX, SPEED_MIN

logger = logging.getLogger(__name__)


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Roll, pitch, yaw (rad) → quaternion x, y, z, w (same convention as tf)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)

def _quat_conj(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_mul(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Quaternion multiplication (x,y,z,w) ⊗ (x,y,z,w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


# Rotation that maps NED basis to ENU basis used above:
# (x_north, y_east, z_down) -> (x_enu=y_east, y_enu=x_north, z_enu=-z_down)
# This is a 180° rotation about axis (1,1,0)/sqrt(2).
_Q_NED_TO_ENU: Tuple[float, float, float, float] = (
    1.0 / math.sqrt(2.0),
    1.0 / math.sqrt(2.0),
    0.0,
    0.0,
)


def _quat_ned_to_enu(q_ned: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Convert orientation from NED world frame to ENU world frame."""
    t = _Q_NED_TO_ENU
    return _quat_mul(_quat_mul(t, q_ned), _quat_conj(t))


def _ned_to_enu(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """NED → ENU for RViz (x_north→y, y_east→x, z_down→-z)."""
    return (float(y), float(x), float(-z))


def _row_to_pose_stamped(
    row: Dict[str, Any],
    node: Node,
    frame_id: str = "world",
) -> PoseStamped:
    enu_x, enu_y, enu_z = _ned_to_enu(
        float(row["x"]), float(row["y"]), float(row["z"])
    )
    qx, qy, qz, qw = _quaternion_from_euler(
        float(row["rx"]), float(row["ry"]), float(row["rz"])
    )
    qx, qy, qz, qw = _quat_ned_to_enu((qx, qy, qz, qw))
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = enu_x
    msg.pose.position.y = enu_y
    msg.pose.position.z = enu_z
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


# Distinct colors for markers (RGBA 0..1); cycles if num_drones > len
_MARKER_PALETTE: List[Tuple[float, float, float]] = [
    (0.9, 0.2, 0.2),
    (0.2, 0.6, 0.9),
    (0.3, 0.85, 0.3),
    (0.95, 0.75, 0.2),
    (0.75, 0.35, 0.85),
    (0.2, 0.85, 0.85),
    (0.95, 0.45, 0.55),
    (0.55, 0.55, 0.95),
]


def _default_iris_mesh_path() -> str:
    """Bundled Collada mesh next to the repo (visualizer/models/iris.dae)."""
    return os.path.join(_replay_parent, "visualizer", "models", "iris.dae")


def _mesh_resource_uri(local_path: str) -> str:
    """Absolute path → file:// URI for Marker.mesh_resource (POSIX)."""
    ap = os.path.abspath(os.path.expanduser(local_path))
    if ap.startswith(os.sep):
        return "file://" + ap
    return "file:///" + ap.replace(os.sep, "/")


def _rows_to_marker_array(
    step_list: List[Optional[Dict[str, Any]]],
    node: Node,
    frame_id: str,
    *,
    mesh_resource_uri: Optional[str] = None,
    mesh_scale: float = 1.0,
    sphere_scale: float = 0.45,
    ground: bool = False,
    ground_size_m: float = 200.0,
    ground_z_m: float = 0.0,
) -> MarkerArray:
    """Build MarkerArray for RViz2: optional Collada mesh (e.g. iris.dae) or colored spheres."""
    out = MarkerArray()
    stamp = node.get_clock().now().to_msg()
    n_palette = len(_MARKER_PALETTE)
    use_mesh = bool(mesh_resource_uri)
    ms = float(mesh_scale)
    if ms <= 0:
        ms = 1.0
    if ground:
        s = max(1.0, float(ground_size_m))
        z0 = float(ground_z_m)
        gm = Marker()
        gm.header.frame_id = frame_id
        gm.header.stamp = stamp
        gm.ns = "environment"
        gm.id = 0
        gm.action = Marker.ADD
        gm.type = Marker.CUBE
        gm.pose.position.x = 0.0
        gm.pose.position.y = 0.0
        # Put the top face exactly at z0 (ENU up).
        gm.pose.position.z = z0 - 0.01
        gm.pose.orientation.w = 1.0
        gm.scale.x = s
        gm.scale.y = s
        gm.scale.z = 0.02
        # "Grass" color.
        gm.color = ColorRGBA(r=0.20, g=0.60, b=0.20, a=1.0)
        gm.lifetime.sec = 0
        gm.lifetime.nanosec = 0
        out.markers.append(gm)
    for i, row in enumerate(step_list):
        if row is None:
            continue
        drone_id = i + 1
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "swarm"
        m.id = drone_id
        m.action = Marker.ADD
        enu_x, enu_y, enu_z = _ned_to_enu(
            float(row["x"]), float(row["y"]), float(row["z"])
        )
        qx, qy, qz, qw = _quaternion_from_euler(
            float(row["rx"]), float(row["ry"]), float(row["rz"])
        )
        qx, qy, qz, qw = _quat_ned_to_enu((qx, qy, qz, qw))
        m.pose.position.x = enu_x
        m.pose.position.y = enu_y
        m.pose.position.z = enu_z
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw
        collision = int(row.get("hasCollision", 0))
        if use_mesh:
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = mesh_resource_uri or ""
            m.mesh_use_embedded_materials = collision == 0
            m.scale.x = ms
            m.scale.y = ms
            m.scale.z = ms
            if collision:
                m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
            else:
                m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        else:
            m.type = Marker.SPHERE
            m.scale.x = sphere_scale
            m.scale.y = sphere_scale
            m.scale.z = sphere_scale
            pr, pg, pb = _MARKER_PALETTE[(drone_id - 1) % n_palette]
            col = ColorRGBA(r=pr, g=pg, b=pb, a=1.0)
            if collision:
                col = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
            m.color = col
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        out.markers.append(m)
    return out


def _lerp_angle(a: float, b: float, t: float) -> float:
    """Кратчайшая интерполяция угла (рад)."""
    d = ((b - a + math.pi) % (2 * math.pi)) - math.pi
    return a + d * t


def _blend_row(
    a: Optional[Dict[str, Any]],
    b: Optional[Dict[str, Any]],
    alpha: float,
) -> Optional[Dict[str, Any]]:
    """Линейная смесь двух строк CSV (NED поза + углы). alpha=0 → a, 1 → b."""
    if a is None:
        return b
    if b is None:
        return a
    u = 1.0 - alpha
    hc = int(b["hasCollision"]) if alpha >= 0.999 else int(a["hasCollision"])
    return {
        "t": float(a["t"]) * u + float(b["t"]) * alpha,
        "x": float(a["x"]) * u + float(b["x"]) * alpha,
        "y": float(a["y"]) * u + float(b["y"]) * alpha,
        "z": float(a["z"]) * u + float(b["z"]) * alpha,
        "rx": _lerp_angle(float(a["rx"]), float(b["rx"]), alpha),
        "ry": _lerp_angle(float(a["ry"]), float(b["ry"]), alpha),
        "rz": _lerp_angle(float(a["rz"]), float(b["rz"]), alpha),
        "hasCollision": hc,
    }


def _interpolate_step_lists(
    s0: List[Optional[Dict[str, Any]]],
    s1: List[Optional[Dict[str, Any]]],
    alpha: float,
) -> List[Optional[Dict[str, Any]]]:
    """Покадровая смесь двух шагов (по индексу дрона)."""
    n = max(len(s0), len(s1))
    out: List[Optional[Dict[str, Any]]] = []
    for i in range(n):
        a = s0[i] if i < len(s0) else None
        b = s1[i] if i < len(s1) else None
        out.append(_blend_row(a, b, alpha))
    return out


def _publish_step_poses_and_markers(
    node: Node,
    step_list: List[Optional[Dict[str, Any]]],
    pose_pubs: Dict[int, Any],
    markers_pub: Optional[Any],
    frame_id: str,
    publish_markers: bool,
    *,
    marker_mesh_uri: Optional[str] = None,
    marker_mesh_scale: float = 1.0,
    marker_sphere_scale: float = 0.45,
    ground: bool = False,
    ground_size_m: float = 200.0,
    ground_z_m: float = 0.0,
) -> None:
    for i, row in enumerate(step_list):
        if row is None:
            continue
        drone_id = i + 1
        pose_pubs[drone_id].publish(_row_to_pose_stamped(row, node, frame_id=frame_id))
    if publish_markers and markers_pub is not None:
        markers_pub.publish(
            _rows_to_marker_array(
                step_list,
                node,
                frame_id,
                mesh_resource_uri=marker_mesh_uri,
                mesh_scale=marker_mesh_scale,
                sphere_scale=marker_sphere_scale,
                ground=ground,
                ground_size_m=ground_size_m,
                ground_z_m=ground_z_m,
            )
        )


def _keyboard_loop(state: PlaybackState, step_times: List[float], shutdown_event: threading.Event) -> None:
    if not sys.stdin.isatty():
        return
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not shutdown_event.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            key = sys.stdin.read(1)
            if key == " ":
                state.toggle_playing()
            elif key == "q" or key == "\x03":
                shutdown_event.set()
                break
            elif key in ("+", "="):
                state.adjust_speed(0.25)
            elif key == "-":
                state.adjust_speed(-0.25)
            elif key == "\x1b":
                more = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0.05)[0] else ""
                seq = key + more
                if seq == "\x1b[D":
                    state.set_index(state.get_state()[1] - 1)
                elif seq == "\x1b[C":
                    state.set_index(state.get_state()[1] + 1)
            elif key == "a":
                state.set_index(state.get_state()[1] - 1)
            elif key == "d":
                state.set_index(state.get_state()[1] + 1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _timeline_ui_thread(
    state: PlaybackState,
    step_times: List[float],
    shutdown_event: threading.Event,
    title: str = "Swarm Replay",
) -> None:
    """Optional local UI (Tk slider + play/pause) for interactive replay."""
    try:
        import tkinter as tk
        from tkinter import ttk
    except Exception as e:
        logger.warning(
            "Timeline UI недоступен (tkinter import failed: %s). "
            "На Ubuntu обычно решается пакетом python3-tk.",
            e,
        )
        return

    n = len(step_times)
    if n <= 0:
        return

    root = tk.Tk()
    root.title(title)
    root.geometry("520x160")

    def on_close() -> None:
        shutdown_event.set()
        try:
            root.destroy()
        except Exception:
            pass

    def toggle_play() -> None:
        state.toggle_playing()

    def step_prev() -> None:
        playing, idx, _ = state.get_state()
        if playing:
            state.set_playing(False)
        state.set_index(idx - 1)

    def step_next() -> None:
        playing, idx, _ = state.get_state()
        if playing:
            state.set_playing(False)
        state.set_index(idx + 1)

    dragging = {"active": False}

    def on_slider_press(_: object = None) -> None:
        dragging["active"] = True

    def on_slider_release(_: object = None) -> None:
        dragging["active"] = False

    def on_slider_changed(v: str) -> None:
        if not dragging["active"]:
            return
        try:
            state.set_index(int(float(v)))
        except Exception:
            return

    frm = ttk.Frame(root, padding=10)
    frm.pack(fill="both", expand=True)

    status_var = tk.StringVar(value="")
    ttk.Label(frm, textvariable=status_var).pack(anchor="w")

    scale = ttk.Scale(
        frm,
        from_=0,
        to=max(0, n - 1),
        orient="horizontal",
        command=on_slider_changed,
    )
    scale.pack(fill="x", pady=10)
    scale.bind("<ButtonPress-1>", on_slider_press)
    scale.bind("<ButtonRelease-1>", on_slider_release)

    btns = ttk.Frame(frm)
    btns.pack(fill="x")
    ttk.Button(btns, text="⟵ Step", command=step_prev).pack(side="left")
    ttk.Button(btns, text="Play/Pause", command=toggle_play).pack(side="left", padx=8)
    ttk.Button(btns, text="Step ⟶", command=step_next).pack(side="left")
    ttk.Button(btns, text="Stop", command=on_close).pack(side="right")

    root.protocol("WM_DELETE_WINDOW", on_close)

    def refresh() -> None:
        if shutdown_event.is_set():
            on_close()
            return
        playing, idx, speed = state.get_state()
        idx = max(0, min(n - 1, idx))
        t = step_times[idx] if idx < len(step_times) else 0.0
        t0 = step_times[0] if step_times else 0.0
        t1 = step_times[-1] if step_times else 0.0
        status_var.set(
            f"{'PLAY' if playing else 'PAUSE'}  "
            f"idx={idx+1}/{n}  t={t:.3f}s  speed={speed:.2f}x  duration={max(0.0, t1 - t0):.3f}s"
        )
        if not dragging["active"]:
            try:
                scale.set(idx)
            except Exception:
                pass
        root.after(100, refresh)

    root.after(100, refresh)
    try:
        root.mainloop()
    except Exception:
        shutdown_event.set()


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Replay experiment in RViz 2 (ROS 2).")
    p.add_argument(
        "--experiment",
        type=str,
        required=True,
        help="Experiment directory (metadata.json + drone_*.csv).",
    )
    p.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Playback speed (1.0 = real-time). With --interactive, initial speed.",
    )
    p.add_argument(
        "--interactive",
        action="store_true",
        default=True,
        help=(
            "Интерактивный режим (по умолчанию включён): play/pause, seek, speed через клавиатуру и /replay/* topics. "
            "Чтобы выключить: --no-interactive."
        ),
    )
    p.add_argument(
        "--no-interactive",
        action="store_false",
        dest="interactive",
        help="Выключить интерактивный режим (линейное воспроизведение без управления).",
    )
    p.add_argument(
        "--start-paused",
        action="store_true",
        help="Только с --interactive: старт на паузе (Space — play). По умолчанию сразу идёт воспроизведение.",
    )
    p.add_argument(
        "--timeline-ui",
        action="store_true",
        default=True,
        help="Только с --interactive: открыть локальное окно с таймлайном (ползунок) и кнопками Play/Pause.",
    )
    p.add_argument(
        "--no-timeline-ui",
        action="store_false",
        dest="timeline_ui",
        help="Выключить локальное окно таймлайна (ползунок).",
    )
    p.add_argument(
        "--frame-id",
        type=str,
        default="world",
        help="PoseStamped header frame_id (RViz2 Fixed Frame must match).",
    )
    p.add_argument(
        "--no-markers",
        action="store_true",
        help="Do not publish /swarm/markers (only per-drone /swarm/drone_N/pose).",
    )
    p.add_argument(
        "--marker-spheres",
        action="store_true",
        help="Использовать цветные сферы вместо меша (по умолчанию — iris.dae из visualizer/models).",
    )
    p.add_argument(
        "--marker-mesh",
        type=str,
        default="",
        metavar="PATH",
        help=(
            "Путь к .dae/.stl для Marker.MESH_RESOURCE. Пусто = visualizer/models/iris.dae "
            "(если файл есть), иначе сферы."
        ),
    )
    p.add_argument(
        "--marker-mesh-scale",
        type=float,
        default=1.0,
        metavar="S",
        help="Равномерный масштаб меша (RViz scale.x/y/z). По умолчанию 1.0.",
    )
    p.add_argument(
        "--marker-sphere-scale",
        type=float,
        default=0.45,
        metavar="S",
        help="Диаметр/масштаб сферы при --marker-spheres (метры).",
    )
    p.add_argument(
        "--ground",
        action="store_true",
        default=True,
        help="Добавить плоскость-«землю» (зелёный тонкий куб) в /swarm/markers (по умолчанию включено).",
    )
    p.add_argument(
        "--no-ground",
        action="store_false",
        dest="ground",
        help="Выключить плоскость-«землю».",
    )
    p.add_argument(
        "--ground-size",
        type=float,
        default=50.0,
        metavar="M",
        help="Размер стороны «земли» (метры). По умолчанию 50.",
    )
    p.add_argument(
        "--ground-z",
        type=float,
        default=0.0,
        metavar="Z",
        help="Высота плоскости «земли» в ENU (метры). По умолчанию 0.",
    )
    p.add_argument(
        "--rviz",
        action="store_true",
        help="Запустить RViz2 с replay/rviz2_swarm_replay.rviz (сетка + MarkerArray /swarm/markers, Fixed Frame world).",
    )
    p.add_argument(
        "--rviz-config",
        type=str,
        default="",
        help=(
            "Путь к .rviz. По умолчанию: replay/rviz2_swarm_replay_sky.rviz (если есть), "
            "иначе replay/rviz2_swarm_replay.rviz рядом со скриптом."
        ),
    )
    p.add_argument(
        "--viz-substeps",
        type=int,
        default=1,
        metavar="N",
        help=(
            "Сглаживание: между двумя записями лога публиковать N промежуточных кадров. "
            "1 = только CSV. Большие N + высокий FPS в RViz дают джиттер; начните с 3–4 или используйте --viz-cap-hz."
        ),
    )
    p.add_argument(
        "--viz-cap-hz",
        type=float,
        default=0.0,
        metavar="HZ",
        help=(
            "Ограничить среднюю частоту публикаций (Гц). При малом шаге лога (например 50 Гц → dt≈0.02 с) "
            "низкий лимит срезает --viz-substeps до 1 — движение скачками. "
            "0 = без лимита (полное сглаживание). Пример: --viz-substeps 8 --viz-cap-hz 0"
        ),
    )
    p.add_argument(
        "--viz-spatial-step-m",
        type=float,
        default=0.02,
        metavar="M",
        help=(
            "Целевой горизонтальный шаг (м) между визуальными кадрами: при большой скорости "
            "автоматически увеличивает число подшагов (до 96), даже если --viz-substeps 1. "
            "0 = отключить. По умолчанию 0.02 (2 см)."
        ),
    )
    return p.parse_args()


def _warn_if_viz_cap_clamps_substeps(
    steps: List[Tuple[float, List[Optional[Dict[str, Any]]]]],
    viz_substeps: int,
    viz_cap_hz: float,
    rate: float,
) -> None:
    """Log when --viz-cap-hz forces effective substeps to 1 so motion looks like discrete log jumps."""
    if viz_cap_hz <= 0 or viz_substeps <= 1 or rate <= 0 or len(steps) < 2:
        return
    sample = min(200, len(steps) - 1)
    dts = [steps[i + 1][0] - steps[i][0] for i in range(sample)]
    dts = [d for d in dts if d > 1e-9]
    if not dts:
        return
    dt_med = sorted(dts)[len(dts) // 2]
    seg_req = max(1, int(viz_substeps))
    max_seg = max(1, int(viz_cap_hz * dt_med / rate))
    eff = min(seg_req, max_seg)
    if eff < seg_req:
        logger.warning(
            "С --viz-cap-hz=%.1f и типичным шагом времени в логе ≈%.4f с (rate=%.2f) интерполяция "
            "срезана до %d кадр(ов) между точками лога (запрошено --viz-substeps=%d). "
            "Движение будет рваным; для сглаживания задайте --viz-cap-hz 0 или увеличьте лимит "
            "(нужно примерно ≥ %.0f Гц при этом dt).",
            viz_cap_hz,
            dt_med,
            rate,
            eff,
            seg_req,
            seg_req * rate / dt_med if dt_med > 0 else 0.0,
        )


def _effective_substeps(
    requested: int,
    dt_log: float,
    rate: float,
    viz_cap_hz: float,
) -> int:
    """Сколько сегментов интерполяции использовать при лимите частоты публикаций."""
    seg = max(1, int(requested))
    if viz_cap_hz <= 0 or rate <= 0:
        return seg
    dt_log = max(dt_log, 1e-9)
    # seg публикаций за время dt_log/rate → средняя частота seg*rate/dt_log
    max_seg = int(viz_cap_hz * dt_log / rate)
    max_seg = max(1, max_seg)
    return min(seg, max_seg)


def _max_horiz_step_ned_m(
    s0: List[Optional[Dict[str, Any]]],
    s1: List[Optional[Dict[str, Any]]],
) -> float:
    """Максимум по дронам горизонтального шага |Δx,Δy| (NED) между двумя выровненными шагами."""
    m = 0.0
    n = min(len(s0), len(s1))
    for i in range(n):
        a, b = s0[i], s1[i]
        if a is None or b is None:
            continue
        dx = float(b["x"]) - float(a["x"])
        dy = float(b["y"]) - float(a["y"])
        d = math.hypot(dx, dy)
        if d > m:
            m = d
    return m


def _segment_substeps(
    nseg_base: int,
    s0: List[Optional[Dict[str, Any]]],
    s1: List[Optional[Dict[str, Any]]],
    dt: float,
    rate: float,
    viz_cap_hz: float,
    spatial_step_m: float,
) -> int:
    """Сколько подшагов между двумя записями лога: не меньше запрошенного и не меньше чем по дистанции."""
    req = max(1, int(nseg_base))
    if spatial_step_m > 0:
        dmax = _max_horiz_step_ned_m(s0, s1)
        if dmax > 1e-9:
            need = int(math.ceil(dmax / spatial_step_m))
            need = max(1, min(96, need))
            req = max(req, need)
    return _effective_substeps(req, dt, rate, viz_cap_hz)


def _launch_rviz2(config_path: str, wait_sec: float = 1.5) -> None:
    """Start rviz2 -d config in background (non-blocking)."""
    config_path = os.path.abspath(config_path)
    if not os.path.isfile(config_path):
        logger.warning("Файл конфигурации RViz не найден: %s", config_path)
        return
    exe = shutil.which("rviz2")
    if not exe:
        logger.warning("rviz2 не найден в PATH (установите ros-jazzy-desktop и source setup.bash).")
        return
    try:
        subprocess.Popen([exe, "-d", config_path], start_new_session=True)
        logger.info("Запущен RViz2: %s -d %s", exe, config_path)
        time.sleep(wait_sec)
    except OSError as e:
        logger.warning("Не удалось запустить rviz2: %s", e)


def _run_linear(
    node: Node,
    steps: List[Tuple[float, List[Optional[Dict[str, Any]]]]],
    pose_pubs: Dict[int, Any],
    markers_pub: Optional[Any],
    rate: float,
    frame_id: str,
    publish_markers: bool,
    viz_substeps: int,
    viz_cap_hz: float,
    spatial_step_m: float = 0.0,
    *,
    marker_mesh_uri: Optional[str] = None,
    marker_mesh_scale: float = 1.0,
    marker_sphere_scale: float = 0.45,
    ground: bool = False,
    ground_size_m: float = 200.0,
    ground_z_m: float = 0.0,
) -> None:
    nseg = max(1, int(viz_substeps))
    use_interp = nseg > 1 or spatial_step_m > 0
    if not use_interp:
        prev_t = 0.0
        for t, step_list in steps:
            if not rclpy.ok():
                break
            _publish_step_poses_and_markers(
                node,
                step_list,
                pose_pubs,
                markers_pub,
                frame_id,
                publish_markers,
                marker_mesh_uri=marker_mesh_uri,
                marker_mesh_scale=marker_mesh_scale,
                marker_sphere_scale=marker_sphere_scale,
                ground=ground,
                ground_size_m=ground_size_m,
                ground_z_m=ground_z_m,
            )
            dt = t - prev_t
            prev_t = t
            if dt > 0 and rate > 0:
                time.sleep(dt / rate)
            rclpy.spin_once(node, timeout_sec=0.0)
        logger.info("Replay finished.")
        return

    if not steps:
        return
    _publish_step_poses_and_markers(
        node,
        steps[0][1],
        pose_pubs,
        markers_pub,
        frame_id,
        publish_markers,
        marker_mesh_uri=marker_mesh_uri,
        marker_mesh_scale=marker_mesh_scale,
        marker_sphere_scale=marker_sphere_scale,
        ground=ground,
        ground_size_m=ground_size_m,
        ground_z_m=ground_z_m,
    )
    rclpy.spin_once(node, timeout_sec=0.0)
    for i in range(len(steps) - 1):
        if not rclpy.ok():
            break
        _t0, s0 = steps[i]
        t1, s1 = steps[i + 1]
        dt = max(t1 - _t0, 1e-9)
        seg = _segment_substeps(nseg, s0, s1, dt, rate, viz_cap_hz, spatial_step_m)
        for k in range(1, seg + 1):
            if not rclpy.ok():
                break
            alpha = k / seg
            sl = _interpolate_step_lists(s0, s1, alpha)
            _publish_step_poses_and_markers(
                node,
                sl,
                pose_pubs,
                markers_pub,
                frame_id,
                publish_markers,
                marker_mesh_uri=marker_mesh_uri,
                marker_mesh_scale=marker_mesh_scale,
                marker_sphere_scale=marker_sphere_scale,
                ground=ground,
                ground_size_m=ground_size_m,
                ground_z_m=ground_z_m,
            )
            if rate > 0:
                time.sleep(dt / seg / rate)
            rclpy.spin_once(node, timeout_sec=0.0)
    logger.info(
        "Replay finished (viz-substeps=%d, spatial-step-m=%s).",
        nseg,
        spatial_step_m if spatial_step_m > 0 else "off",
    )


def _run_interactive(
    node: Node,
    steps: List[Tuple[float, List[Optional[Dict[str, Any]]]]],
    pose_pubs: Dict[int, Any],
    markers_pub: Optional[Any],
    meta_pub: Any,
    metadata: Dict[str, Any],
    state: PlaybackState,
    frame_id: str,
    publish_markers: bool,
    use_keyboard: bool,
    shutdown_event: threading.Event,
    viz_substeps: int,
    viz_cap_hz: float,
    spatial_step_m: float = 0.0,
    *,
    marker_mesh_uri: Optional[str] = None,
    marker_mesh_scale: float = 1.0,
    marker_sphere_scale: float = 0.45,
    ground: bool = False,
    ground_size_m: float = 200.0,
    ground_z_m: float = 0.0,
) -> None:
    step_times = [s[0] for s in steps]
    n = len(steps)
    nseg = max(1, int(viz_substeps))
    use_seg_interp = nseg > 1 or spatial_step_m > 0

    def on_play(_: Empty) -> None:
        state.set_playing(True)

    def on_pause(_: Empty) -> None:
        state.set_playing(False)

    def on_seek(msg: Float64) -> None:
        state.seek_to_time(float(msg.data), step_times)

    def on_speed(msg: Float32) -> None:
        state.set_speed(float(msg.data))

    node.create_subscription(Empty, "/replay/play", on_play, 10)
    node.create_subscription(Empty, "/replay/pause", on_pause, 10)
    node.create_subscription(Float64, "/replay/seek", on_seek, 10)
    node.create_subscription(Float32, "/replay/speed", on_speed, 10)

    if use_keyboard:
        threading.Thread(
            target=_keyboard_loop,
            args=(state, step_times, shutdown_event),
            daemon=True,
        ).start()
        logger.info(
            "Keyboard: Space=play/pause, a/d or arrows=step, +/-=speed, q=quit"
        )

    meta = String()
    meta.data = json.dumps(metadata)
    meta_pub.publish(meta)

    while rclpy.ok() and not shutdown_event.is_set():
        playing, idx, speed = state.get_state()
        t, step_list = steps[idx]

        if playing and idx < n - 1 and use_seg_interp:
            s0 = step_list
            s1 = steps[idx + 1][1]
            dt = max(steps[idx + 1][0] - t, 1e-9)
            seg = _segment_substeps(nseg, s0, s1, dt, speed, viz_cap_hz, spatial_step_m)
            for k in range(1, seg + 1):
                if not rclpy.ok() or shutdown_event.is_set():
                    break
                alpha = k / seg
                sl = _interpolate_step_lists(s0, s1, alpha)
                _publish_step_poses_and_markers(
                    node,
                    sl,
                    pose_pubs,
                    markers_pub,
                    frame_id,
                    publish_markers,
                    marker_mesh_uri=marker_mesh_uri,
                    marker_mesh_scale=marker_mesh_scale,
                    marker_sphere_scale=marker_sphere_scale,
                    ground=ground,
                    ground_size_m=ground_size_m,
                    ground_z_m=ground_z_m,
                )
                if speed > 0:
                    time.sleep(dt / seg / speed)
                rclpy.spin_once(node, timeout_sec=0.0)
            state.advance_index()
        else:
            _publish_step_poses_and_markers(
                node,
                step_list,
                pose_pubs,
                markers_pub,
                frame_id,
                publish_markers,
                marker_mesh_uri=marker_mesh_uri,
                marker_mesh_scale=marker_mesh_scale,
                marker_sphere_scale=marker_sphere_scale,
                ground=ground,
                ground_size_m=ground_size_m,
                ground_z_m=ground_z_m,
            )
            if playing and idx < n - 1:
                next_t = steps[idx + 1][0]
                dt = max(next_t - t, 1e-9)
                if speed > 0:
                    time.sleep(dt / speed)
                state.advance_index()
            else:
                if playing and idx >= n - 1:
                    state.set_playing(False)
                time.sleep(0.05)
            rclpy.spin_once(node, timeout_sec=0.0)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = _parse_args()
    rate = args.rate
    if rate <= 0:
        logger.error("--rate must be positive.")
        sys.exit(1)
    rate = max(SPEED_MIN, min(SPEED_MAX, rate))

    try:
        data = load_experiment(args.experiment)
    except (ValueError, OSError) as e:
        logger.error("Failed to load experiment: %s", e)
        sys.exit(1)

    metadata = data.get("metadata") or {}
    drone_logs = data.get("drone_logs") or []
    if not drone_logs:
        logger.error("No drone CSV files in: %s", args.experiment)
        sys.exit(1)

    num_drones = len(drone_logs)
    steps = list(iter_steps(loaded=data, align="t"))
    if not steps:
        logger.error("No steps from iter_steps.")
        sys.exit(1)

    raw_vs = int(args.viz_substeps)
    if raw_vs < 1:
        logger.warning("--viz-substeps должно быть >= 1, используется 1.")
    viz_substeps = max(1, raw_vs)
    viz_cap_hz = max(0.0, float(args.viz_cap_hz))
    spatial_step_m = max(0.0, float(getattr(args, "viz_spatial_step_m", 0.0)))

    marker_sphere_scale = max(0.01, float(args.marker_sphere_scale))
    marker_mesh_scale = max(0.01, float(args.marker_mesh_scale))
    ground_enabled = bool(getattr(args, "ground", False))
    ground_size_m = float(getattr(args, "ground_size", 200.0))
    ground_z_m = float(getattr(args, "ground_z", 0.0))
    marker_mesh_uri: Optional[str] = None
    if args.marker_spheres:
        logger.info("Маркеры RViz: сферы (масштаб %.3f м).", marker_sphere_scale)
    else:
        mesh_path = (args.marker_mesh or "").strip() or _default_iris_mesh_path()
        if os.path.isfile(mesh_path):
            marker_mesh_uri = _mesh_resource_uri(mesh_path)
            logger.info(
                "Маркеры RViz: mesh %s (uniform scale %.3f).",
                mesh_path,
                marker_mesh_scale,
            )
        else:
            logger.warning(
                "Файл меша не найден (%s) — используются сферы. "
                "Положите iris.dae в visualizer/models или задайте --marker-mesh.",
                mesh_path,
            )

    rclpy.init()
    node = Node("swarm_replay")
    shutdown_event = threading.Event()

    qos_scale = max(viz_substeps, 48 if spatial_step_m > 0 else 1)
    qos_depth = max(10, min(200, 10 * qos_scale))
    qos = QoSProfile(
        depth=qos_depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    pose_pubs: Dict[int, Any] = {}
    for i in range(1, num_drones + 1):
        pose_pubs[i] = node.create_publisher(
            PoseStamped, f"/swarm/drone_{i}/pose", qos
        )
    meta_pub = node.create_publisher(String, "/swarm/metadata", 1)
    publish_markers = not args.no_markers
    markers_pub = None
    if publish_markers:
        markers_pub = node.create_publisher(MarkerArray, "/swarm/markers", qos)

    default_rviz_config = os.path.join(_script_dir, "rviz2_swarm_replay_sky.rviz")
    if not os.path.isfile(default_rviz_config):
        default_rviz_config = os.path.join(_script_dir, "rviz2_swarm_replay.rviz")
    rviz_config = args.rviz_config or default_rviz_config
    if args.rviz:
        if args.frame_id != "world":
            logger.warning(
                "Конфиг RViz задан с Fixed Frame=world; у вас --frame-id=%s — при несовпадении "
                "обновите Fixed Frame в RViz или отредактируйте .rviz.",
                args.frame_id,
            )
        if not publish_markers:
            logger.warning(
                "С --no-markers дисплей MarkerArray в конфиге будет пуст; добавьте Pose вручную или уберите --no-markers."
            )
        _launch_rviz2(rviz_config)

    def _shutdown(sig=None, frame=None) -> None:
        shutdown_event.set()

    signal.signal(signal.SIGINT, _shutdown)

    logger.info(
        "ROS 2 replay: %d drones, %d steps, rate=%.2f, interactive=%s, frame_id=%s, viz-substeps=%d",
        num_drones,
        len(steps),
        rate,
        args.interactive,
        args.frame_id,
        viz_substeps,
    )
    if viz_substeps > 1:
        logger.info(
            "Сглаживание: до %d кадров между точками лога (очередь QoS depth=%d).",
            viz_substeps,
            qos_depth,
        )
    if viz_cap_hz > 0:
        logger.info(
            "Лимит публикаций: ~%.1f Гц (меньше джиттера в RViz при большом --viz-substeps).",
            viz_cap_hz,
        )
        _warn_if_viz_cap_clamps_substeps(steps, viz_substeps, viz_cap_hz, rate)
    if spatial_step_m > 0:
        logger.info(
            "Авто-подшаги по дистанции: цель ≤ %.3f м на кадр (макс. 96 между записями лога); "
            "отключить: --viz-spatial-step-m 0",
            spatial_step_m,
        )
    if publish_markers:
        logger.info(
            "RViz2: Fixed Frame = %s → Add → By topic → /swarm/markers (MarkerArray) "
            "— все %d дронов подключатся одним дисплеем.",
            args.frame_id,
            num_drones,
        )
    logger.info(
        "Опционально: отдельные топики /swarm/drone_1/pose … /swarm/drone_%d/pose",
        num_drones,
    )

    try:
        if args.interactive:
            state = PlaybackState(
                num_steps=len(steps),
                initial_speed=rate,
                start_paused=args.start_paused,
            )
            if args.timeline_ui:
                step_times = [s[0] for s in steps]
                threading.Thread(
                    target=_timeline_ui_thread,
                    args=(state, step_times, shutdown_event),
                    daemon=True,
                ).start()
            if args.start_paused:
                logger.info(
                    "Интерактив: старт на паузе — нажмите Space в терминале replay для воспроизведения."
                )
            else:
                logger.info(
                    "Интерактив: воспроизведение включено; Space — пауза/продолжение."
                )
            _run_interactive(
                node,
                steps,
                pose_pubs,
                markers_pub,
                meta_pub,
                metadata,
                state,
                args.frame_id,
                publish_markers,
                use_keyboard=sys.stdin.isatty(),
                shutdown_event=shutdown_event,
                viz_substeps=viz_substeps,
                viz_cap_hz=viz_cap_hz,
                spatial_step_m=spatial_step_m,
                marker_mesh_uri=marker_mesh_uri,
                marker_mesh_scale=marker_mesh_scale,
                marker_sphere_scale=marker_sphere_scale,
                ground=ground_enabled,
                ground_size_m=ground_size_m,
                ground_z_m=ground_z_m,
            )
        else:
            m = String()
            m.data = json.dumps(metadata)
            meta_pub.publish(m)
            # Allow subscribers to connect
            end = time.time() + 1.0
            while time.time() < end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
            _run_linear(
                node,
                steps,
                pose_pubs,
                markers_pub,
                rate,
                args.frame_id,
                publish_markers,
                viz_substeps=viz_substeps,
                viz_cap_hz=viz_cap_hz,
                spatial_step_m=spatial_step_m,
                marker_mesh_uri=marker_mesh_uri,
                marker_mesh_scale=marker_mesh_scale,
                marker_sphere_scale=marker_sphere_scale,
                ground=ground_enabled,
                ground_size_m=ground_size_m,
                ground_z_m=ground_z_m,
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

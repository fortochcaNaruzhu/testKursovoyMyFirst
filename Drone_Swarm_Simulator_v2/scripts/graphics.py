#!/usr/bin/env python3
"""
Графики по папке эксперимента (metadata.json + drone_*.csv).

Требует расширенный CSV (19 колонок) для σ/PWM/скоростей — как от antena_logic_copy / fntena_logic_copy2.
Дистанции до якоря (таргета) строятся по позиции дрона-якоря из того же эксперимента.

Пример:
  python scripts/graphics.py --experiment /path/to/exp_dir --focus-drone 2
  python scripts/graphics.py --experiment /path/to/exp_dir --save-dir /path/to/plots
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# Неинтерактивный backend по умолчанию (удобно по SSH).
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

_project_root = Path(__file__).resolve().parent.parent
if str(_project_root) not in sys.path:
    sys.path.insert(0, str(_project_root))

# Согласовано с replay/csv_loader и csv_logger
HEADER_LEGACY_8 = "t,x,y,z,rx,ry,rz,hasCollision"
HEADER_9 = "t,x,y,z,rx,ry,rz,hasCollision,sitl_time_boot_s"
HEADER_19 = (
    HEADER_9
    + ",sigma_x,sigma_y,sigma_z,rc_roll,rc_pitch,rc_throttle,rc_yaw,vx,vy,vz"
)


def _load_metadata(exp_dir: Path) -> Dict[str, Any]:
    p = exp_dir / "metadata.json"
    if not p.is_file():
        raise FileNotFoundError(f"Нет metadata.json: {p}")
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)


def _expected_cols(header_line: str) -> int:
    h = header_line.strip()
    if h == HEADER_LEGACY_8:
        return 8
    if h == HEADER_9:
        return 9
    if h == HEADER_19:
        return 19
    raise ValueError(
        f"Неизвестный заголовок CSV (ожидалось 8, 9 или 19 колонок): {h[:120]}..."
    )


def _load_drone_csv(path: Path) -> Tuple[int, List[List[str]]]:
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        n = _expected_cols(",".join(header))
        rows: List[List[str]] = []
        for row in reader:
            if not row or all(not c.strip() for c in row):
                continue
            if len(row) != n:
                raise ValueError(f"{path}: ожидалось {n} колонок, строка имеет {len(row)}")
            rows.append(row)
    return n, rows


def _rows_to_arrays(
    ncols: int, rows: List[List[str]]
) -> Dict[str, np.ndarray]:
    if not rows:
        return {
            "t": np.array([]),
            "x": np.array([]),
            "y": np.array([]),
            "z": np.array([]),
        }
    data = np.array(rows, dtype=float)
    out: Dict[str, np.ndarray] = {
        "t": data[:, 0],
        "x": data[:, 1],
        "y": data[:, 2],
        "z": data[:, 3],
    }
    if ncols >= 19:
        out["sigma_x"] = data[:, 9]
        out["sigma_y"] = data[:, 10]
        out["sigma_z"] = data[:, 11]
        out["rc_roll"] = data[:, 12]
        out["rc_pitch"] = data[:, 13]
        out["rc_throttle"] = data[:, 14]
        out["rc_yaw"] = data[:, 15]
        out["vx"] = data[:, 16]
        out["vy"] = data[:, 17]
        out["vz"] = data[:, 18]
    return out


def _interp_nearest_sorted(
    t_query: np.ndarray, t_ref: np.ndarray, y_ref: np.ndarray
) -> np.ndarray:
    """Линейная интерполяция; t_ref должен быть отсортирован по возрастанию."""
    if t_ref.size == 0:
        return np.full_like(t_query, np.nan, dtype=float)
    return np.interp(t_query, t_ref, y_ref, left=np.nan, right=np.nan)


def _distances_to_anchor(
    drone: Dict[str, np.ndarray], anchor: Dict[str, np.ndarray]
) -> Tuple[np.ndarray, np.ndarray]:
    """Горизонтальная дистанция в плоскости XY и |Δz| до якоря (NED), по времени дрона."""
    t = drone["t"]
    ta = anchor["t"]
    order = np.argsort(ta)
    ta_s = ta[order]
    xa_s = anchor["x"][order]
    ya_s = anchor["y"][order]
    za_s = anchor["z"][order]
    ax = _interp_nearest_sorted(t, ta_s, xa_s)
    ay = _interp_nearest_sorted(t, ta_s, ya_s)
    az = _interp_nearest_sorted(t, ta_s, za_s)
    d_xy = np.sqrt((drone["x"] - ax) ** 2 + (drone["y"] - ay) ** 2)
    d_z = np.abs(drone["z"] - az)
    return d_xy, d_z


def main() -> None:
    parser = argparse.ArgumentParser(description="Графики по эксперименту (CSV + metadata.json).")
    parser.add_argument(
        "--experiment",
        type=str,
        required=True,
        help="Каталог эксперимента (metadata.json + drone_*.csv).",
    )
    parser.add_argument(
        "--focus-drone",
        type=int,
        default=1,
        help="Номер дрона для графиков σ, PWM и скоростей (по умолчанию 1).",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="",
        help="Если задано — сохранить PNG в эту папку (иначе показывать не будет: используйте Agg и см. файлы).",
    )
    parser.add_argument("--show", action="store_true", help="Показать окна matplotlib (нужен дисплей).")
    args = parser.parse_args()

    exp_dir = Path(os.path.abspath(args.experiment))
    meta = _load_metadata(exp_dir)
    num_drones = int(meta.get("num_drones", 0))
    if num_drones < 1:
        raise ValueError("metadata.json: num_drones должно быть >= 1")
    anchor_id = int(meta.get("anchor_id", 1))
    if anchor_id < 1 or anchor_id > num_drones:
        anchor_id = 1

    drones_data: Dict[int, Tuple[int, Dict[str, np.ndarray]]] = {}
    for i in range(1, num_drones + 1):
        csv_path = exp_dir / f"drone_{i}.csv"
        if not csv_path.is_file():
            raise FileNotFoundError(f"Нет файла {csv_path}")
        ncols, rows = _load_drone_csv(csv_path)
        drones_data[i] = (ncols, _rows_to_arrays(ncols, rows))

    _, anchor_arr = drones_data[anchor_id]

    if args.save_dir:
        save_dir: Optional[Path] = Path(args.save_dir).resolve()
    elif args.show:
        save_dir = None
    else:
        save_dir = (exp_dir / "graphics_plots").resolve()
    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)

    if args.show:
        plt.switch_backend("TkAgg")

    colors = plt.cm.tab10(np.linspace(0, 1, max(num_drones, 2)))

    # --- 1) XY distance vs time (все дроны) ---
    fig1, ax1 = plt.subplots(figsize=(10, 5))
    for idx, did in enumerate(range(1, num_drones + 1)):
        _, arr = drones_data[did]
        if arr["t"].size == 0:
            continue
        d_xy, _ = _distances_to_anchor(arr, anchor_arr)
        ax1.plot(arr["t"], d_xy, color=colors[idx % len(colors)], label=f"drone {did}", lw=1.2)
    ax1.set_xlabel("t, s")
    ax1.set_ylabel("distance XY to anchor, m")
    ax1.set_title(f"Horizontal distance to anchor (drone {anchor_id})")
    ax1.legend(loc="best", fontsize=8)
    ax1.grid(True, alpha=0.3)
    fig1.tight_layout()
    if save_dir:
        fig1.savefig(save_dir / "dist_xy_to_anchor.png", dpi=150)

    # --- 2) |Δz| vs time (все дроны) ---
    fig2, ax2 = plt.subplots(figsize=(10, 5))
    for idx, did in enumerate(range(1, num_drones + 1)):
        _, arr = drones_data[did]
        if arr["t"].size == 0:
            continue
        _, d_z = _distances_to_anchor(arr, anchor_arr)
        ax2.plot(arr["t"], d_z, color=colors[idx % len(colors)], label=f"drone {did}", lw=1.2)
    ax2.set_xlabel("t, s")
    ax2.set_ylabel("|Δz| to anchor, m (NED z)")
    ax2.set_title(f"Vertical separation |z - z_anchor|, anchor drone {anchor_id}")
    ax2.legend(loc="best", fontsize=8)
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    if save_dir:
        fig2.savefig(save_dir / "dist_dz_to_anchor.png", dpi=150)

    focus = int(args.focus_drone)
    if focus < 1 or focus > num_drones:
        raise ValueError(f"--focus-drone должен быть 1..{num_drones}, получено {focus}")
    ncols_f, focus_arr = drones_data[focus]
    if ncols_f < 19:
        print(
            f"Предупреждение: drone_{focus}.csv без телеметрии (колонок {ncols_f}, нужно 19). "
            "Графики σ/PWM/скоростей пропущены.",
            file=sys.stderr,
        )
    else:
        t_f = focus_arr["t"]
        # --- 3) все sigma ---
        fig3, ax3 = plt.subplots(figsize=(10, 5))
        ax3.plot(t_f, focus_arr["sigma_x"], label="sigma_x", lw=1.2)
        ax3.plot(t_f, focus_arr["sigma_y"], label="sigma_y", lw=1.2)
        ax3.plot(t_f, focus_arr["sigma_z"], label="sigma_z", lw=1.2)
        ax3.set_xlabel("t, s")
        ax3.set_ylabel("sigma")
        ax3.set_title(f"Sigma vs time (drone {focus})")
        ax3.legend(loc="best")
        ax3.grid(True, alpha=0.3)
        fig3.tight_layout()
        if save_dir:
            fig3.savefig(save_dir / f"sigma_drone_{focus}.png", dpi=150)

        # --- 4) все PWM ---
        fig4, ax4 = plt.subplots(figsize=(10, 5))
        ax4.plot(t_f, focus_arr["rc_roll"], label="roll", lw=1.0)
        ax4.plot(t_f, focus_arr["rc_pitch"], label="pitch", lw=1.0)
        ax4.plot(t_f, focus_arr["rc_throttle"], label="throttle", lw=1.0)
        ax4.plot(t_f, focus_arr["rc_yaw"], label="yaw", lw=1.0)
        ax4.set_xlabel("t, s")
        ax4.set_ylabel("PWM")
        ax4.set_title(f"RC channels vs time (drone {focus})")
        ax4.legend(loc="best")
        ax4.grid(True, alpha=0.3)
        fig4.tight_layout()
        if save_dir:
            fig4.savefig(save_dir / f"pwm_drone_{focus}.png", dpi=150)

        # --- 5) скорости NED ---
        fig5, ax5 = plt.subplots(figsize=(10, 5))
        ax5.plot(t_f, focus_arr["vx"], label="vx", lw=1.2)
        ax5.plot(t_f, focus_arr["vy"], label="vy", lw=1.2)
        ax5.plot(t_f, focus_arr["vz"], label="vz (down +)", lw=1.2)
        ax5.set_xlabel("t, s")
        ax5.set_ylabel("velocity, m/s")
        ax5.set_title(f"NED velocity vs time (drone {focus})")
        ax5.legend(loc="best")
        ax5.grid(True, alpha=0.3)
        fig5.tight_layout()
        if save_dir:
            fig5.savefig(save_dir / f"velocity_drone_{focus}.png", dpi=150)

    if args.show:
        plt.show()
    if save_dir is not None:
        print(f"Сохранено в {save_dir}")


if __name__ == "__main__":
    main()

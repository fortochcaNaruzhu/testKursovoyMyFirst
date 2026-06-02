#!/usr/bin/env python3
"""
Сравнение двух прогонов mavlink_compare: «чистый» vs «с помехами».

Строит наложенные кривые ошибки оценки GLOBAL_POSITION_INT относительно SIM_STATE
(слитые поля в одной строке CSV после mavlink_compare_logger).

Метрики:
  — горизонтальная ошибка положения (м), haversine(sim_lat/lon, gpi_lat/lon);
  — ошибка высоты gpi_alt − sim_alt (м);
  — модуль ошибки скорости в NED (м/с);
  — ошибка курса: gpi_hdg_deg − deg(sim_yaw), в диапазоне ±180°.

Полные roll/pitch оценки EKF в этом CSV нет (нет ATTITUDE); курс и скорость
отражают связанную с ориентацией/навигацией ошибку оценки.

Ожидаемое поведение фильтра: при «нормальных» и зашумлённых измерениях веса
взвешенного согласования (EKF и т.п.) не дают ошибке неограниченно расти —
типичны конечный разброс и смещение, пока модель и шумы согласованы и есть
Выход: для каждого дрона два PNG — наложение (`compare_gpi_vs_sim_drone_N.png`)
и два столбца с общей шкалой по строке (`compare_gpi_vs_sim_drone_N_by_run.png`).

Пример:
  source ../drone_env/bin/activate
  python scripts/plot_mavlink_compare_clean_vs_noisy.py \\
    --clean-dir ../../test_logov/antena_NOshum_run/mavlink_compare \\
    --noisy-dir ../../test_logov/antena_shum_run/mavlink_compare \\
    --output-dir ../../test_logov/mavlink_compare_clean_vs_noisy_plots
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_R_EARTH_M = 6_371_000.0


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    a = min(1.0, max(0.0, a))
    return 2.0 * _R_EARTH_M * math.asin(math.sqrt(a))


def _angle_diff_deg(a_deg: float, b_deg: float) -> float:
    d = (a_deg - b_deg + 180.0) % 360.0 - 180.0
    return d


def _f(x: str) -> Optional[float]:
    s = (x or "").strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def _nanrms(y: np.ndarray) -> float:
    a = np.asarray(y, dtype=np.float64)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(a * a)))


def _nanmax_abs(y: np.ndarray) -> float:
    a = np.asarray(y, dtype=np.float64)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return float("nan")
    return float(np.max(np.abs(a)))


def _nanmax_nonneg(y: np.ndarray) -> float:
    a = np.asarray(y, dtype=np.float64)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return float("nan")
    return float(np.max(a))


def _stats_box_overlay(ax, y_c: np.ndarray, y_n: np.ndarray, signed: bool) -> None:
    if signed:
        ext_c = f"max|…|={_nanmax_abs(y_c):.4g}"
        ext_n = f"max|…|={_nanmax_abs(y_n):.4g}"
    else:
        ext_c = f"max={_nanmax_nonneg(y_c):.4g}"
        ext_n = f"max={_nanmax_nonneg(y_n):.4g}"
    txt = (
        f"без помех: RMS={_nanrms(y_c):.4g}; {ext_c}\n"
        f"с помехами: RMS={_nanrms(y_n):.4g}; {ext_n}"
    )
    ax.text(
        0.01,
        0.99,
        txt,
        transform=ax.transAxes,
        va="top",
        fontsize=7,
        linespacing=1.05,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.82, edgecolor="0.7"),
    )


def _stats_box_single(ax, y: np.ndarray, signed: bool) -> None:
    if signed:
        ext = f"max|…|={_nanmax_abs(y):.4g}"
    else:
        ext = f"max={_nanmax_nonneg(y):.4g}"
    txt = f"RMS={_nanrms(y):.4g}; {ext}"
    ax.text(
        0.01,
        0.99,
        txt,
        transform=ax.transAxes,
        va="top",
        fontsize=7,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.82, edgecolor="0.7"),
    )


def load_global_errors(csv_path: Path) -> Dict[str, np.ndarray]:
    """Из строк GLOBAL_POSITION_INT извлекает t_rel и ошибки относительно SIM."""
    t: List[float] = []
    horiz: List[float] = []
    alt_e: List[float] = []
    vel_e: List[float] = []
    hdg_e: List[float] = []

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if (row.get("msg_type") or "").strip() != "GLOBAL_POSITION_INT":
                continue
            slat = _f(row.get("sim_lat_deg", "") or "")
            slon = _f(row.get("sim_lon_deg", "") or "")
            salt = _f(row.get("sim_alt_m", "") or "")
            glat = _f(row.get("gpi_lat_deg", "") or "")
            glon = _f(row.get("gpi_lon_deg", "") or "")
            galt = _f(row.get("gpi_alt_m", "") or "")
            if None in (slat, slon, salt, glat, glon, galt):
                continue
            tr = _f(row.get("t_rel_s", "") or "")
            if tr is None:
                continue

            svn = _f(row.get("sim_vn_m_s", "") or "")
            sve = _f(row.get("sim_ve_m_s", "") or "")
            svd = _f(row.get("sim_vd_m_s", "") or "")
            gvx = _f(row.get("gpi_vx_m_s", "") or "")
            gvy = _f(row.get("gpi_vy_m_s", "") or "")
            gvz = _f(row.get("gpi_vz_m_s", "") or "")
            syaw = _f(row.get("sim_yaw_rad", "") or "")
            ghdg = _f(row.get("gpi_hdg_deg", "") or "")

            t.append(tr)
            horiz.append(_haversine_m(slat, slon, glat, glon))
            alt_e.append(galt - salt)

            if None not in (svn, sve, svd, gvx, gvy, gvz):
                dvx, dvy, dvz = gvx - svn, gvy - sve, gvz - svd
                vel_e.append(math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz))
            else:
                vel_e.append(float("nan"))

            if syaw is not None and ghdg is not None:
                yaw_deg = math.degrees(syaw)
                hdg_e.append(_angle_diff_deg(ghdg, yaw_deg))
            else:
                hdg_e.append(float("nan"))

    return {
        "t_rel_s": np.array(t, dtype=np.float64),
        "horiz_m": np.array(horiz, dtype=np.float64),
        "alt_m": np.array(alt_e, dtype=np.float64),
        "vel_ms": np.array(vel_e, dtype=np.float64),
        "hdg_deg": np.array(hdg_e, dtype=np.float64),
    }


def _plot_pair(
    ax,
    t_c: np.ndarray,
    y_c: np.ndarray,
    t_n: np.ndarray,
    y_n: np.ndarray,
    ylabel: str,
    title: str,
    *,
    signed: bool,
) -> None:
    ax.plot(t_c, y_c, color="#2ca02c", lw=0.8, alpha=0.9, label="без помех")
    ax.plot(t_n, y_n, color="#d62728", lw=0.8, alpha=0.9, label="с помехами")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    _stats_box_overlay(ax, y_c, y_n, signed=signed)


def _row_ylim_symmetric(y0: np.ndarray, y1: np.ndarray, pct: float = 99.0) -> Tuple[float, float]:
    a = np.concatenate(
        [np.asarray(y0, dtype=np.float64), np.asarray(y1, dtype=np.float64)]
    )
    a = a[np.isfinite(a)]
    if a.size == 0:
        return -1.0, 1.0
    m = float(np.nanpercentile(np.abs(a), pct))
    m = max(m, 1e-9)
    return -m, m


def _row_ylim_nonneg(y0: np.ndarray, y1: np.ndarray, pct: float = 99.5) -> Tuple[float, float]:
    a = np.concatenate(
        [np.asarray(y0, dtype=np.float64), np.asarray(y1, dtype=np.float64)]
    )
    a = a[np.isfinite(a)]
    if a.size == 0:
        return 0.0, 1.0
    hi = float(np.nanpercentile(a, pct))
    hi = max(hi, 1e-9)
    return 0.0, hi * 1.05


def plot_drone_side_by_side(
    drone_id: int,
    dc: Dict[str, np.ndarray],
    dn: Dict[str, np.ndarray],
    out_path: Path,
) -> None:
    """Два столбца: только без шума / только с шумом; общая шкала Y по строке."""
    rows = [
        ("horiz_m", "м", "Горизонтальная ошибка положения", "nonneg"),
        ("alt_m", "м", "Ошибка высоты (GPI − SIM)", "sym"),
        ("vel_ms", "м/с", "Ошибка скорости ‖GPI v − SIM v‖", "nonneg"),
        ("hdg_deg", "град", "Ошибка курса (GPI hdg − SIM yaw)", "sym"),
    ]
    fig, axes = plt.subplots(4, 2, figsize=(11, 10), sharex="col")
    fig.suptitle(
        f"Дрон {drone_id}: одна шкала по строке — слева без помех, справа с помехами"
    )
    for i, (key, ylab, ttl, mode) in enumerate(rows):
        yc, yn = dc[key], dn[key]
        tc, tn = dc["t_rel_s"], dn["t_rel_s"]
        axes[i, 0].plot(tc, yc, color="#2ca02c", lw=0.75)
        axes[i, 1].plot(tn, yn, color="#d62728", lw=0.75)
        axes[i, 0].set_ylabel(ylab)
        axes[i, 0].set_title(f"{ttl} — без помех")
        axes[i, 1].set_title(f"{ttl} — с помехами")
        axes[i, 0].grid(True, alpha=0.3)
        axes[i, 1].grid(True, alpha=0.3)
        if mode == "sym":
            lo, hi = _row_ylim_symmetric(yc, yn)
            axes[i, 0].set_ylim(lo, hi)
            axes[i, 1].set_ylim(lo, hi)
        else:
            lo, hi = _row_ylim_nonneg(yc, yn)
            axes[i, 0].set_ylim(lo, hi)
            axes[i, 1].set_ylim(lo, hi)
        _stats_box_single(axes[i, 0], yc, signed=(mode == "sym"))
        _stats_box_single(axes[i, 1], yn, signed=(mode == "sym"))
    axes[-1, 0].set_xlabel("t_rel, с")
    axes[-1, 1].set_xlabel("t_rel, с")
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_drone(
    drone_id: int,
    clean_csv: Path,
    noisy_csv: Path,
    out_overlay: Path,
    out_side_by_side: Path,
) -> None:
    dc = load_global_errors(clean_csv)
    dn = load_global_errors(noisy_csv)

    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    fig.suptitle(
        f"Дрон {drone_id}: ошибка оценки (GPI) относительно SIM "
        f"(наложение; RMS/max в рамках — конечные величины)"
    )

    _plot_pair(
        axes[0, 0],
        dc["t_rel_s"],
        dc["horiz_m"],
        dn["t_rel_s"],
        dn["horiz_m"],
        "м",
        "Горизонтальная ошибка положения",
        signed=False,
    )
    _plot_pair(
        axes[0, 1],
        dc["t_rel_s"],
        dc["alt_m"],
        dn["t_rel_s"],
        dn["alt_m"],
        "м",
        "Ошибка высоты (GPI alt − SIM alt)",
        signed=True,
    )
    _plot_pair(
        axes[1, 0],
        dc["t_rel_s"],
        dc["vel_ms"],
        dn["t_rel_s"],
        dn["vel_ms"],
        "м/с",
        "Ошибка скорости (‖GPI v − SIM v‖, NED)",
        signed=False,
    )
    # heading: mask nan
    yc, yn = dc["hdg_deg"], dn["hdg_deg"]
    _plot_pair(
        axes[1, 1],
        dc["t_rel_s"],
        yc,
        dn["t_rel_s"],
        yn,
        "град",
        "Ошибка курса (GPI hdg − SIM yaw)",
        signed=True,
    )

    axes[1, 0].set_xlabel("t_rel, с")
    axes[1, 1].set_xlabel("t_rel, с")
    fig.tight_layout()
    out_overlay.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_overlay, dpi=150)
    plt.close(fig)

    plot_drone_side_by_side(drone_id, dc, dn, out_side_by_side)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean-dir", type=Path, required=True)
    ap.add_argument("--noisy-dir", type=Path, required=True)
    ap.add_argument("--output-dir", type=Path, required=True)
    ap.add_argument("--drones", type=str, default="1,2,3,4", help="список id через запятую")
    args = ap.parse_args()

    clean_dir: Path = args.clean_dir.resolve()
    noisy_dir: Path = args.noisy_dir.resolve()
    out_dir: Path = args.output_dir.resolve()
    ids = [int(x.strip()) for x in args.drones.split(",") if x.strip()]

    for did in ids:
        c = clean_dir / f"drone_{did}_mavlink_compare.csv"
        n = noisy_dir / f"drone_{did}_mavlink_compare.csv"
        if not c.is_file():
            print(f"Пропуск дрон {did}: нет {c}", file=sys.stderr)
            continue
        if not n.is_file():
            print(f"Пропуск дрон {did}: нет {n}", file=sys.stderr)
            continue
        plot_drone(
            did,
            c,
            n,
            out_dir / f"compare_gpi_vs_sim_drone_{did}.png",
            out_dir / f"compare_gpi_vs_sim_drone_{did}_by_run.png",
        )
        print(f"OK: {out_dir / f'compare_gpi_vs_sim_drone_{did}.png'}")
        print(f"OK: {out_dir / f'compare_gpi_vs_sim_drone_{did}_by_run.png'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

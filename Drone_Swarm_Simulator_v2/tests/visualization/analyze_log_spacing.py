#!/usr/bin/env python3
"""
Анализ частоты записи CSV и шага координат между соседними строками (в мм).

Режимы:
  pair       — two_drones_log_*.csv (колонки leader_x/y, follower_x/y, …)
  experiment — каталог с drone_1.csv, drone_2.csv, … (заголовок t,x,y,z,…)

Пример (из корня Drone_Swarm_Simulator_v2):
  python3 tests/visualization/analyze_log_spacing.py pair logs/two_drones_log_run_exchange_sync.csv
  python3 tests/visualization/analyze_log_spacing.py experiment logs/replay_from_run
  python3 tests/visualization/analyze_log_spacing.py pair logs/foo.csv -o tests/visualization/out/report.txt
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple


def _pct(sorted_vals: Sequence[float], p: float) -> float:
    if not sorted_vals:
        return float("nan")
    n = len(sorted_vals)
    if n == 1:
        return float(sorted_vals[0])
    i = min(n - 1, max(0, int(round(p * (n - 1)))))
    return float(sorted_vals[i])


def _summarize(name: str, values: List[float], unit: str) -> str:
    if not values:
        return f"{name}: нет данных"
    s = sorted(values)
    m = sum(s) / len(s)
    lines = [
        f"{name} (n={len(s)}, {unit}):",
        f"  min={s[0]:.6g}  max={s[-1]:.6g}  mean={m:.6g}  median={_pct(s, 0.5):.6g}",
        f"  p90={_pct(s, 0.9):.6g}  p99={_pct(s, 0.99):.6g}",
    ]
    if len(s) > 1:
        var = sum((x - m) ** 2 for x in s) / (len(s) - 1)
        lines.append(f"  stdev={math.sqrt(var):.6g}")
    return "\n".join(lines)


def _delta_times(ts: List[float]) -> Tuple[List[float], int]:
    dts: List[float] = []
    bad = 0
    for i in range(1, len(ts)):
        d = ts[i] - ts[i - 1]
        if d <= 0:
            bad += 1
        else:
            dts.append(d)
    return dts, bad


def _horiz_mm(
    x0: float, y0: float, x1: float, y1: float,
) -> float:
    return math.hypot(x1 - x0, y1 - y0) * 1000.0


def _analyze_pair(path: Path) -> str:
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return "Пустой CSV"
        fields = [h.strip() for h in reader.fieldnames]
        need = ("t", "leader_x", "leader_y", "follower_x", "follower_y")
        for k in need:
            if k not in fields:
                return f"Нет колонки {k!r}. Есть: {fields}"
        rows = list(reader)

    ts = [float(r["t"]) for r in rows]
    lx = [float(r["leader_x"]) for r in rows]
    ly = [float(r["leader_y"]) for r in rows]
    fx = [float(r["follower_x"]) for r in rows]
    fy = [float(r["follower_y"]) for r in rows]

    dts, bad_dt = _delta_times(ts)
    leader_mm: List[float] = []
    follower_mm: List[float] = []
    leader_zero = 0
    follower_zero = 0
    for i in range(1, len(rows)):
        lm = _horiz_mm(lx[i - 1], ly[i - 1], lx[i], ly[i])
        fm = _horiz_mm(fx[i - 1], fy[i - 1], fx[i], fy[i])
        leader_mm.append(lm)
        follower_mm.append(fm)
        if lm < 1e-3:
            leader_zero += 1
        if fm < 1e-3:
            follower_zero += 1

    out: List[str] = [
        f"Файл: {path}",
        f"Строк данных: {len(rows)}  t: {ts[0]:.6f} … {ts[-1]:.6f} с (длительность {ts[-1]-ts[0]:.3f} с)",
        "",
        "### Время между соседними строками (с)",
    ]
    if bad_dt:
        out.append(f"Предупреждение: пропущено интервалов с dt<=0: {bad_dt}")
    out.append(_summarize("Δt", dts, "s"))
    if dts:
        inv = [1.0 / d for d in dts]
        out.append("")
        out.append(_summarize("Мгновенная частота 1/Δt", inv, "Hz"))

    out.extend(
        [
            "",
            "### Горизонтальный шаг между соседними строками (мм, |Δx,Δy| в плоскости NED)",
            _summarize("Лидер (leader_x, leader_y)", leader_mm, "mm"),
            "",
            _summarize("Фолловер (follower_x, follower_y)", follower_mm, "mm"),
            "",
            f"Строк с шагом лидера < 0.001 мм (практически тот же кадр): {leader_zero} / {max(0, len(rows)-1)}",
            f"Строк с шагом фолловера < 0.001 мм: {follower_zero} / {max(0, len(rows)-1)}",
        ]
    )
    return "\n".join(out)


def _analyze_experiment_dir(exp_dir: Path) -> str:
    csvs = sorted(exp_dir.glob("drone_*.csv"))
    if not csvs:
        return f"В {exp_dir} нет drone_*.csv"

    blocks: List[str] = [f"Каталог: {exp_dir}", ""]
    for p in csvs:
        with open(p, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames is None:
                blocks.append(f"{p.name}: пусто")
                continue
            rows = list(reader)
        if not rows:
            blocks.append(f"{p.name}: нет строк")
            continue
        ts = [float(r["t"]) for r in rows]
        xs = [float(r["x"]) for r in rows]
        ys = [float(r["y"]) for r in rows]
        dts, bad_dt = _delta_times(ts)
        mm: List[float] = []
        for i in range(1, len(rows)):
            mm.append(_horiz_mm(xs[i - 1], ys[i - 1], xs[i], ys[i]))
        blocks.append(f"## {p.name}")
        blocks.append(f"Строк: {len(rows)}  t: {ts[0]:.6f} … {ts[-1]:.6f} с")
        if bad_dt:
            blocks.append(f"Предупреждение: dt<=0: {bad_dt}")
        blocks.append(_summarize("Δt", dts, "s"))
        if dts:
            inv = [1.0 / d for d in dts]
            blocks.append(_summarize("1/Δt", inv, "Hz"))
        blocks.append(_summarize("Горизонтальный шаг (x,y) мм", mm, "mm"))
        blocks.append("")
    return "\n".join(blocks).rstrip() + "\n"


def main() -> None:
    p = argparse.ArgumentParser(description="Анализ Δt и шага координат в мм в логах.")
    p.add_argument(
        "mode",
        choices=("pair", "experiment"),
        help="pair: CSV пары лидер/фолловер; experiment: каталог с drone_*.csv",
    )
    p.add_argument("path", type=str, help="Путь к CSV (pair) или к каталогу эксперимента")
    p.add_argument(
        "-o",
        "--output",
        type=str,
        default="",
        help="Записать отчёт в файл (каталог должен существовать или будет создан для родителя)",
    )
    args = p.parse_args()
    target = Path(args.path).resolve()
    if args.mode == "pair":
        if not target.is_file():
            print(f"Файл не найден: {target}", file=sys.stderr)
            sys.exit(1)
        report = _analyze_pair(target)
    else:
        if not target.is_dir():
            print(f"Каталог не найден: {target}", file=sys.stderr)
            sys.exit(1)
        report = _analyze_experiment_dir(target)

    print(report)
    if args.output:
        out_path = Path(args.output).resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(report, encoding="utf-8")
        print(f"\n[Записано: {out_path}]", file=sys.stderr)


if __name__ == "__main__":
    main()

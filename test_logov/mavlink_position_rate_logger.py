#!/usr/bin/env python3
"""
Log raw MAVLink message arrival times to measure effective rate.

This connects directly to MAVProxy/SITL telemetry (udp:127.0.0.1:<port>), optionally requests
message intervals, then writes every received message to CSV.

IMPORTANT (Drone_Swarm_Simulator_v2): use the logger ``tap`` UDP ports, not the scenario ports.
The launcher duplicates ``--out`` as base port (MAVLinkWorker) and base+100 (passive loggers).
Example: drone 1 → scenario 14551, logger 14651; drone 2 → 14561 / 14661; etc.

Default telemetry matches the simulator: SIM_STATE (pose + attitude + vn/ve/vd), with
HOME_POSITION latched for NED x,y,z columns (same convention as ``core/mavlink/worker.py``).

With --include-attitude, ATTITUDE messages are also logged (interleaved) for comparison.

Use --legacy-local-ned to log LOCAL_POSITION_NED instead (older CSV/plot workflows).

CSV columns:
  wall_time, msg_type, dt_wall_any, dt_wall_type, time_boot_ms,
  mode, custom_mode, base_mode, hb_sysid, hb_compid,
  x,y,z,vx,vy,vz, roll,pitch,yaw

Use this to diagnose "plateaus and jumps" caused by sparse / bursty MAVLink updates.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from typing import Any, Dict, Optional, Tuple

from pymavlink import mavutil

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_SIM_PKG = os.path.join(_REPO_ROOT, "Drone_Swarm_Simulator_v2")
if _SIM_PKG not in sys.path:
    sys.path.insert(0, _SIM_PKG)

from core.mavlink.geo_ned import (  # noqa: E402
    home_position_lat_lon_alt_m,
    ned_metres_from_home,
    sim_state_lat_lon_deg,
)


def _sim_state_msg_id() -> int:
    return int(getattr(mavutil.mavlink, "MAVLINK_MSG_ID_SIM_STATE", 108))


def _home_position_msg_id() -> int:
    return int(getattr(mavutil.mavlink, "MAVLINK_MSG_ID_HOME_POSITION", 242))


def _hb_sys_comp(msg: Any) -> Tuple[int, int]:
    try:
        return (int(msg.get_srcSystem()), int(msg.get_srcComponent()))
    except Exception:
        pass
    try:
        return (int(getattr(msg, "srcSystem", 0)), int(getattr(msg, "srcComponent", 0)))
    except Exception:
        return (0, 0)


def _request_interval(master: Any, msg_id: int, hz: float) -> None:
    if hz <= 0:
        return
    interval_us = max(1, int(1e6 / float(hz)))
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def _mode_string(master: Any, hb_msg: Any) -> str:
    try:
        s = mavutil.mode_string_v10(hb_msg)
        if s:
            return str(s)
    except Exception:
        pass
    try:
        cm = int(getattr(hb_msg, "custom_mode", -1))
        return f"custom_mode_{cm}"
    except Exception:
        return "unknown"


def main() -> None:
    ap = argparse.ArgumentParser(description="Measure MAVLink message effective rate.")
    ap.add_argument("--port", type=int, default=14551, help="SITL MAVLink UDP port (default 14551).")
    ap.add_argument("--duration", type=float, default=20.0, help="Seconds to log (default 20).")
    ap.add_argument(
        "--hz",
        type=float,
        default=0.0,
        help=(
            "If >0, send MAV_CMD_SET_MESSAGE_INTERVAL: SIM_STATE at this Hz (default mode), "
            "plus HOME_POSITION at 2 Hz for NED origin. "
            "With --legacy-local-ned, requests LOCAL_POSITION_NED instead. "
            "Default 0 = do not send; use ArduPilot stream params only."
        ),
    )
    ap.add_argument("--att-hz", type=float, default=0.0, help="Optional ATTITUDE requested rate (Hz).")
    ap.add_argument(
        "--include-attitude",
        action="store_true",
        help="Also log ATTITUDE messages (interleaved) into one CSV.",
    )
    ap.add_argument(
        "--legacy-local-ned",
        action="store_true",
        help="Use LOCAL_POSITION_NED instead of SIM_STATE (matches pre–SIM_STATE tooling).",
    )
    ap.add_argument(
        "--include-heartbeat",
        action="store_true",
        help=(
            "Log HEARTBEAT messages as separate CSV rows (msg_type=HEARTBEAT). "
            "Useful to correlate rate degradation with mode changes."
        ),
    )
    ap.add_argument(
        "--out",
        type=str,
        default="test_logov/mavlink_msgs.csv",
        help="Output CSV path (default test_logov/mavlink_msgs.csv).",
    )
    ap.add_argument("--heartbeat-timeout", type=float, default=10.0, help="Seconds to wait for heartbeat.")
    ap.add_argument(
        "--request-heartbeat-hz",
        type=float,
        default=0.0,
        help=(
            "If >0, MAV_CMD_SET_MESSAGE_INTERVAL for HEARTBEAT at this rate (Hz). "
            "Use when CSV mode/custom_mode looks stale vs SITL console (UDP link may carry sparse HEARTBEAT)."
        ),
    )
    args = ap.parse_args()

    conn = f"udp:127.0.0.1:{int(args.port)}"
    master = mavutil.mavlink_connection(conn)
    hb = master.wait_heartbeat(timeout=float(args.heartbeat_timeout))
    if hb is None:
        raise SystemExit(f"No HEARTBEAT on {conn} within {args.heartbeat_timeout}s")

    vehicle_sysid = int(hb.get_srcSystem())

    try:
        master.target_system = vehicle_sysid
        master.target_component = int(hb.get_srcComponent() or 1)
    except Exception:
        pass

    legacy = bool(args.legacy_local_ned)

    if float(args.hz) > 0:
        if legacy:
            _request_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, float(args.hz))
        else:
            _request_interval(master, _home_position_msg_id(), 2.0)
            _request_interval(master, _sim_state_msg_id(), float(args.hz))
    if float(args.att_hz) > 0:
        _request_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, float(args.att_hz))
    if float(args.request_heartbeat_hz) > 0:
        _request_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, float(args.request_heartbeat_hz))
    if float(args.hz) <= 0 and float(args.att_hz) <= 0 and float(args.request_heartbeat_hz) <= 0:
        print(
            "No SET_MESSAGE_INTERVAL sent; observing whatever the FC sends (SR*_POSITION / defaults).",
            flush=True,
        )

    out_path = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    t_end = time.time() + float(args.duration)
    last_wall_any: Optional[float] = None
    last_wall_by_type: Dict[str, float] = {}
    count_by_type: Dict[str, int] = {}
    last_hb: Optional[Any] = hb
    last_mode: str = _mode_string(master, hb)
    last_custom_mode: int = int(getattr(hb, "custom_mode", 0))
    last_base_mode: int = int(getattr(hb, "base_mode", 0))
    last_hb_sys: int
    last_hb_comp: int
    last_hb_sys, last_hb_comp = _hb_sys_comp(hb)

    home_lat_deg: float = 0.0
    home_lon_deg: float = 0.0
    home_alt_m: float = 0.0
    home_initialized: bool = False

    def _sync_mode_from_parser() -> None:
        nonlocal last_mode, last_custom_mode, last_base_mode, last_hb_sys, last_hb_comp
        st = master.sysid_state.get(int(vehicle_sysid))
        if st is None:
            return
        hbm = st.messages.get("HEARTBEAT")
        if hbm is None:
            return
        last_mode = _mode_string(master, hbm)
        last_custom_mode = int(getattr(hbm, "custom_mode", 0))
        last_base_mode = int(getattr(hbm, "base_mode", 0))
        last_hb_sys, last_hb_comp = _hb_sys_comp(hbm)

    def _bump_count(mtype: str) -> None:
        count_by_type[mtype] = int(count_by_type.get(mtype, 0)) + 1

    def _append_row(
        wall_ts: float,
        mtype: str,
        dt_any_v: Any,
        dt_type_v: Any,
        t_boot: Any,
        mode_s: str,
        cust_m: int,
        base_m: int,
        hb_sys: int,
        hb_cmp: int,
        xyzvvv: Tuple[Any, Any, Any, Any, Any, Any],
        rpy: Tuple[Any, Any, Any],
    ) -> None:
        x_, y_, z_, vx_, vy_, vz_ = xyzvvv
        roll_, pitch_, yaw_ = rpy
        w.writerow(
            [
                f"{wall_ts:.6f}",
                mtype,
                (f"{dt_any_v:.6f}" if isinstance(dt_any_v, float) else ""),
                (f"{dt_type_v:.6f}" if isinstance(dt_type_v, float) else ""),
                t_boot,
                mode_s,
                cust_m,
                base_m,
                hb_sys,
                hb_cmp,
                x_,
                y_,
                z_,
                vx_,
                vy_,
                vz_,
                roll_,
                pitch_,
                yaw_,
            ]
        )

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "wall_time",
                "msg_type",
                "dt_wall_any",
                "dt_wall_type",
                "time_boot_ms",
                "mode",
                "custom_mode",
                "base_mode",
                "hb_sysid",
                "hb_compid",
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "roll",
                "pitch",
                "yaw",
            ]
        )
        f.flush()
        while time.time() < t_end:
            if legacy:
                want_types = ["LOCAL_POSITION_NED"]
            else:
                want_types = ["SIM_STATE", "HOME_POSITION"]
            if bool(args.include_attitude):
                want_types.append("ATTITUDE")
            if bool(args.include_heartbeat):
                want_types.append("HEARTBEAT")

            msg = master.recv_match(type=want_types, blocking=True, timeout=1.0)
            if msg is None:
                continue
            now = time.time()
            mtype = msg.get_type()

            _sync_mode_from_parser()

            if mtype == "HEARTBEAT":
                last_hb = msg
                last_mode = _mode_string(master, msg)
                last_custom_mode = int(getattr(msg, "custom_mode", 0))
                last_base_mode = int(getattr(msg, "base_mode", 0))
                last_hb_sys, last_hb_comp = _hb_sys_comp(msg)
                dt_any = (now - last_wall_any) if last_wall_any is not None else ""
                last_wall_any = now
                prev_hb = last_wall_by_type.get("HEARTBEAT")
                dt_hb_only = (now - prev_hb) if prev_hb is not None else ""
                last_wall_by_type["HEARTBEAT"] = now
                _append_row(
                    now,
                    "HEARTBEAT",
                    dt_any,
                    dt_hb_only,
                    "",
                    last_mode,
                    last_custom_mode,
                    last_base_mode,
                    last_hb_sys,
                    last_hb_comp,
                    ("", "", "", "", "", ""),
                    ("", "", ""),
                )
                _bump_count("HEARTBEAT")
                if sum(count_by_type.values()) % 100 == 0:
                    f.flush()
                continue

            if not legacy and mtype == "HOME_POSITION":
                hlat, hlon, halt = home_position_lat_lon_alt_m(msg)
                home_lat_deg = hlat
                home_lon_deg = hlon
                home_alt_m = halt
                home_initialized = True
                continue

            dt_any = (now - last_wall_any) if last_wall_any is not None else ""
            last_wall_any = now
            prev_t = last_wall_by_type.get(mtype)
            dt_type = (now - prev_t) if prev_t is not None else ""
            last_wall_by_type[mtype] = now

            time_boot_ms_v = getattr(msg, "time_boot_ms", None)
            time_boot_ms = int(time_boot_ms_v) if time_boot_ms_v is not None else ""

            x = y = z = vx = vy = vz = ""
            roll = pitch = yaw = ""

            if legacy and mtype == "LOCAL_POSITION_NED":
                x = float(getattr(msg, "x", 0.0))
                y = float(getattr(msg, "y", 0.0))
                z = float(getattr(msg, "z", 0.0))
                vx = float(getattr(msg, "vx", 0.0))
                vy = float(getattr(msg, "vy", 0.0))
                vz = float(getattr(msg, "vz", 0.0))
            elif not legacy and mtype == "SIM_STATE":
                lat, lon = sim_state_lat_lon_deg(msg)
                alt_m = float(getattr(msg, "alt", 0.0))
                if not home_initialized:
                    home_lat_deg = lat
                    home_lon_deg = lon
                    home_alt_m = alt_m
                    home_initialized = True
                x, y, z = ned_metres_from_home(
                    lat, lon, alt_m, home_lat_deg, home_lon_deg, home_alt_m
                )
                vx = float(getattr(msg, "vn", 0.0))
                vy = float(getattr(msg, "ve", 0.0))
                vz = float(getattr(msg, "vd", 0.0))
                roll = float(getattr(msg, "roll", 0.0))
                pitch = float(getattr(msg, "pitch", 0.0))
                yaw = float(getattr(msg, "yaw", 0.0))
            elif mtype == "ATTITUDE":
                roll = float(getattr(msg, "roll", 0.0))
                pitch = float(getattr(msg, "pitch", 0.0))
                yaw = float(getattr(msg, "yaw", 0.0))

            _append_row(
                now,
                mtype,
                dt_any,
                dt_type,
                time_boot_ms,
                last_mode,
                last_custom_mode,
                last_base_mode,
                last_hb_sys,
                last_hb_comp,
                (x, y, z, vx, vy, vz),
                (roll, pitch, yaw),
            )
            _bump_count(mtype)
            if sum(count_by_type.values()) % 100 == 0:
                f.flush()

    parts = [f"{k}={v}" for k, v in sorted(count_by_type.items())]
    print("Wrote messages to %s: %s" % (out_path, ", ".join(parts) if parts else "(none)"))


if __name__ == "__main__":
    main()

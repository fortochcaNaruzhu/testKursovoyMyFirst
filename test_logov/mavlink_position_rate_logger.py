#!/usr/bin/env python3
"""
Log raw MAVLink message arrival times to measure effective rate.

This connects directly to MAVProxy/SITL telemetry (udp:127.0.0.1:<port>), optionally requests
message intervals, then writes every received message to CSV.

IMPORTANT (Drone_Swarm_Simulator_v2): use the logger ``tap`` UDP ports, not the scenario ports.
The launcher duplicates ``--out`` as base port (MAVLinkWorker) and base+100 (passive loggers).
Example: drone 1 → scenario 14551, logger 14651; drone 2 → 14561 / 14661; etc.

When --include-attitude
is enabled, both LOCAL_POSITION_NED and ATTITUDE are logged (interleaved) with a
msg_type column and per-type dt columns:
  wall_time, msg_type, dt_wall_any, dt_wall_type, time_boot_ms,
  mode, custom_mode, base_mode, hb_sysid, hb_compid,
  x,y,z,vx,vy,vz, roll,pitch,yaw

Use this to diagnose "plateaus and jumps" caused by sparse / bursty MAVLink updates.
"""

from __future__ import annotations

import argparse
import csv
import os
import time
from typing import Any, Dict, Optional, Tuple

from pymavlink import mavutil


def _hb_sys_comp(msg: Any) -> Tuple[int, int]:
    try:
        return (int(msg.get_srcSystem()), int(msg.get_srcComponent()))
    except Exception:
        pass
    try:
        return (int(getattr(msg, "srcSystem", 0)), int(getattr(msg, "srcComponent", 0)))
    except Exception:
        return (0, 0)


def _request_interval(master: any, msg_id: int, hz: float) -> None:
    # MAV_CMD_SET_MESSAGE_INTERVAL uses microseconds between messages.
    if hz <= 0:
        return
    interval_us = int(1e6 / float(hz))
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
    """
    Best-effort mode string for ArduPilot.
    Uses pymavlink helpers; falls back to custom_mode integer.
    """
    try:
        s = mavutil.mode_string_v10(hb_msg)
        if s:
            return str(s)
    except Exception:
        pass
    try:
        # master.mode_mapping() may exist, but mapping can be incomplete depending on dialect.
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
            "If >0, send MAV_CMD_SET_MESSAGE_INTERVAL for LOCAL_POSITION_NED at this Hz. "
            "Default 0 = do not send; use ArduPilot stream params only (e.g. SR*_POSITION in iris.parm)."
        ),
    )
    ap.add_argument("--att-hz", type=float, default=0.0, help="Optional ATTITUDE requested rate (Hz).")
    ap.add_argument(
        "--include-attitude",
        action="store_true",
        help="Log both LOCAL_POSITION_NED and ATTITUDE (interleaved) into one CSV.",
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
        help="Output CSV path (default test_logov/mavlink_local_position_ned.csv).",
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

    # Ensure command_long_send targets this autopilot (FC sysid from first vehicle heartbeat).
    try:
        master.target_system = vehicle_sysid
        master.target_component = int(hb.get_srcComponent() or 1)
    except Exception:
        pass

    if float(args.hz) > 0:
        _request_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, float(args.hz))
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
    count_by_type: Dict[str, int] = {"LOCAL_POSITION_NED": 0, "ATTITUDE": 0, "HEARTBEAT": 0}
    last_hb: Optional[Any] = hb
    last_mode: str = _mode_string(master, hb)
    last_custom_mode: int = int(getattr(hb, "custom_mode", 0))
    last_base_mode: int = int(getattr(hb, "base_mode", 0))
    last_hb_sys: int
    last_hb_comp: int
    last_hb_sys, last_hb_comp = _hb_sys_comp(hb)

    def _sync_mode_from_parser() -> None:
        """Pull latest HEARTBEAT fields pymavlink already parsed (recv_match skips non-matching types)."""
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
            want_types = ["LOCAL_POSITION_NED"]
            if bool(args.include_attitude):
                want_types.append("ATTITUDE")
            # Never use a separate recv_match(type='HEARTBEAT') drain: pymavlink drops non-matching
            # messages from recv_match, which steals POSITION/ATTITUDE from this socket.
            if bool(args.include_heartbeat):
                want_types.append("HEARTBEAT")

            msg = master.recv_match(type=want_types, blocking=True, timeout=1.0)
            if msg is None:
                continue
            now = time.time()
            mtype = msg.get_type()

            # INTERPOLATED HEARTBEATs were parsed inside recv_match but not returned; refresh mode.
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
                count_by_type["HEARTBEAT"] = int(count_by_type.get("HEARTBEAT", 0)) + 1
                if sum(count_by_type.values()) % 100 == 0:
                    f.flush()
                continue

            dt_any = (now - last_wall_any) if last_wall_any is not None else ""
            last_wall_any = now
            prev_t = last_wall_by_type.get(mtype)
            dt_type = (now - prev_t) if prev_t is not None else ""
            last_wall_by_type[mtype] = now

            # Extract fields
            time_boot_ms = int(getattr(msg, "time_boot_ms", 0))
            x = y = z = vx = vy = vz = ""
            roll = pitch = yaw = ""
            if mtype == "LOCAL_POSITION_NED":
                x = float(getattr(msg, "x", 0.0))
                y = float(getattr(msg, "y", 0.0))
                z = float(getattr(msg, "z", 0.0))
                vx = float(getattr(msg, "vx", 0.0))
                vy = float(getattr(msg, "vy", 0.0))
                vz = float(getattr(msg, "vz", 0.0))
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
            if mtype in count_by_type:
                count_by_type[mtype] += 1
            else:
                count_by_type[mtype] = 1
            if sum(count_by_type.values()) % 100 == 0:
                f.flush()

    # Quick summary
    parts = [
        "LOCAL_POSITION_NED=%d" % count_by_type.get("LOCAL_POSITION_NED", 0),
        "ATTITUDE=%d" % count_by_type.get("ATTITUDE", 0),
        "HEARTBEAT=%d" % count_by_type.get("HEARTBEAT", 0),
    ]
    if not bool(args.include_attitude):
        parts = [parts[0], parts[2]]
    print("Wrote messages to %s: %s" % (out_path, ", ".join(parts)))


if __name__ == "__main__":
    main()


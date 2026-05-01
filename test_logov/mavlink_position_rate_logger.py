#!/usr/bin/env python3
"""
Log raw MAVLink message arrival times to measure effective rate.

This connects directly to SITL (udp:127.0.0.1:<port>), optionally requests
message intervals, then writes every received message to CSV. When --include-attitude
is enabled, both LOCAL_POSITION_NED and ATTITUDE are logged (interleaved) with a
msg_type column and per-type dt columns:
  wall_time, msg_type, dt_wall_any, dt_wall_type, time_boot_ms, x,y,z,vx,vy,vz, roll,pitch,yaw

Use this to diagnose "plateaus and jumps" caused by sparse / bursty MAVLink updates.
"""

from __future__ import annotations

import argparse
import csv
import os
import time
from typing import Dict, Optional, Tuple

from pymavlink import mavutil


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


def main() -> None:
    ap = argparse.ArgumentParser(description="Measure MAVLink message effective rate.")
    ap.add_argument("--port", type=int, default=14551, help="SITL MAVLink UDP port (default 14551).")
    ap.add_argument("--duration", type=float, default=20.0, help="Seconds to log (default 20).")
    ap.add_argument("--hz", type=float, default=30.0, help="Requested LOCAL_POSITION_NED rate (Hz).")
    ap.add_argument("--att-hz", type=float, default=0.0, help="Optional ATTITUDE requested rate (Hz).")
    ap.add_argument(
        "--include-attitude",
        action="store_true",
        help="Log both LOCAL_POSITION_NED and ATTITUDE (interleaved) into one CSV.",
    )
    ap.add_argument(
        "--out",
        type=str,
        default="test_logov/mavlink_msgs.csv",
        help="Output CSV path (default test_logov/mavlink_local_position_ned.csv).",
    )
    ap.add_argument("--heartbeat-timeout", type=float, default=10.0, help="Seconds to wait for heartbeat.")
    args = ap.parse_args()

    conn = f"udp:127.0.0.1:{int(args.port)}"
    master = mavutil.mavlink_connection(conn)
    hb = master.wait_heartbeat(timeout=float(args.heartbeat_timeout))
    if hb is None:
        raise SystemExit(f"No HEARTBEAT on {conn} within {args.heartbeat_timeout}s")

    # Ensure target system/component are populated for command_long_send.
    try:
        master.target_system = master.target_system or hb.get_srcSystem()
        master.target_component = master.target_component or hb.get_srcComponent()
    except Exception:
        pass

    _request_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, float(args.hz))
    if float(args.att_hz) > 0:
        _request_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, float(args.att_hz))

    out_path = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    t_end = time.time() + float(args.duration)
    last_wall_any: Optional[float] = None
    last_wall_by_type: Dict[str, float] = {}
    count_by_type: Dict[str, int] = {"LOCAL_POSITION_NED": 0, "ATTITUDE": 0}

    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "wall_time",
                "msg_type",
                "dt_wall_any",
                "dt_wall_type",
                "time_boot_ms",
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
            msg = master.recv_match(type=want_types, blocking=True, timeout=1.0)
            if msg is None:
                continue
            now = time.time()
            mtype = msg.get_type()
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
            w.writerow(
                [
                    f"{now:.6f}",
                    mtype,
                    (f"{dt_any:.6f}" if isinstance(dt_any, float) else ""),
                    (f"{dt_type:.6f}" if isinstance(dt_type, float) else ""),
                    time_boot_ms,
                    x,
                    y,
                    z,
                    vx,
                    vy,
                    vz,
                    roll,
                    pitch,
                    yaw,
                ]
            )
            if mtype in count_by_type:
                count_by_type[mtype] += 1
            else:
                count_by_type[mtype] = 1
            if sum(count_by_type.values()) % 100 == 0:
                f.flush()

    # Quick summary
    if bool(args.include_attitude):
        print(
            "Wrote messages to %s: LOCAL_POSITION_NED=%d, ATTITUDE=%d"
            % (out_path, count_by_type.get("LOCAL_POSITION_NED", 0), count_by_type.get("ATTITUDE", 0))
        )
    else:
        print(
            "Wrote messages to %s: LOCAL_POSITION_NED=%d"
            % (out_path, count_by_type.get("LOCAL_POSITION_NED", 0))
        )


if __name__ == "__main__":
    main()


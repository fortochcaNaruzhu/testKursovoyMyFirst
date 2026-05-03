#!/usr/bin/env python3
"""
Dump ALL MAVLink messages from a UDP port to a human-readable .txt file.

Use this on the passive SITL tap2 ports:
  drone1: scenario 14551 → tap 14651 → tap2 14751
  drone2: 14561 → 14661 → 14761, etc.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Any, Dict

from pymavlink import mavutil


def _msg_to_dict(msg: Any) -> Dict[str, Any]:
    try:
        d = msg.to_dict()
        if isinstance(d, dict):
            return d
    except Exception:
        pass
    out: Dict[str, Any] = {"_repr": repr(msg)}
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Dump all MAVLink messages to txt (JSONL).")
    ap.add_argument("--port", type=int, required=True, help="UDP port to listen on (e.g. 14751).")
    ap.add_argument("--duration", type=float, default=60.0, help="Seconds to dump (default 60).")
    ap.add_argument("--out", type=str, required=True, help="Output txt path (JSONL).")
    ap.add_argument("--heartbeat-timeout", type=float, default=15.0, help="Seconds to wait for first HEARTBEAT.")
    args = ap.parse_args()

    out_path = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    conn = f"udp:127.0.0.1:{int(args.port)}"
    master = mavutil.mavlink_connection(conn)
    hb = master.wait_heartbeat(timeout=float(args.heartbeat_timeout))
    if hb is None:
        raise SystemExit(f"No HEARTBEAT on {conn} within {args.heartbeat_timeout}s")

    t_end = time.time() + float(args.duration)
    n = 0
    with open(out_path, "w", encoding="utf-8") as f:
        while time.time() < t_end:
            msg = master.recv_msg()
            if msg is None:
                time.sleep(0.005)
                continue
            if msg.get_type() == "BAD_DATA":
                continue
            now = time.time()
            rec = {
                "wall_time": round(now, 6),
                "sysid": int(msg.get_srcSystem()),
                "compid": int(msg.get_srcComponent()),
                "seq": int(msg.get_seq()),
                "msg_type": str(msg.get_type()),
                "fields": _msg_to_dict(msg),
            }
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            n += 1
            if n % 500 == 0:
                f.flush()

    print(f"Wrote {n} MAVLink messages to {out_path}")


if __name__ == "__main__":
    main()


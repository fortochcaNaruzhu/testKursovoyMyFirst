"""
Solo scenario: connect one drone, verify heartbeat, take off to ~1 meter.
"""

from __future__ import annotations

import logging
import os
import sys
import time
from typing import Dict, Optional

_project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from core.control import DroneController
from core.mavlink.utils import RC_NEUTRAL

logger = logging.getLogger(__name__)

TAKEOFF_ALT_M = 1.0

INIT_STEPS = [
    {"type": "set_mode", "mode_id": 4},  # GUIDED
    {"type": "sleep", "sec": 0.5},
    {"type": "arm"},
    {"type": "sleep", "sec": 2.0},
    {"type": "takeoff"},  # worker uses 1m takeoff in command_long
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


def initialize_drone_solo(
    controller: DroneController,
    init_steps=INIT_STEPS,
    heartbeat_timeout_s: Optional[float] = 12.0,
    position_timeout_s: float = 10.0,
) -> None:
    """
    Same init pattern as linear_chain2: connect -> initialize -> rc_keepalive,
    plus explicit connectivity checks for solo debugging.
    """
    if heartbeat_timeout_s is not None:
        controller.connect_with_heartbeat_timeout(float(heartbeat_timeout_s))
    else:
        controller.connect()
    controller.initialize(list(init_steps))
    controller.start_rc_keepalive()

    # Ensure we actually receive LOCAL_POSITION_NED after requesting the stream.
    t0 = time.time()
    while time.time() - t0 < float(position_timeout_s):
        if controller.get_position() is not None:
            return
        time.sleep(0.1)
    raise TimeoutError("No LOCAL_POSITION_NED received after initialization")


def _wait_for_takeoff(
    controller: DroneController,
    target_alt_m: float = TAKEOFF_ALT_M,
    timeout_s: float = 20.0,
    tol_m: float = 0.30,
) -> bool:
    """
    Wait until LOCAL_POSITION_NED indicates altitude ~ target_alt_m.

    In NED, z is positive down, so up is negative.
    """
    target_z = -float(target_alt_m)
    t0 = time.time()
    last_pos: Optional[Dict[str, float]] = None
    while time.time() - t0 < timeout_s:
        pos = controller.get_position()
        if pos is not None:
            last_pos = pos
            z = float(pos.get("z", 0.0))
            if abs(z - target_z) <= tol_m:
                return True
        time.sleep(0.1)
    logger.warning("Takeoff not confirmed within timeout. Last pos: %s", last_pos)
    return False


def main() -> None:
    import argparse

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    parser = argparse.ArgumentParser(description="Solo: one drone heartbeat + takeoff to 1m")
    # launch_simulation.py always passes --drones; for solo we accept it and require 1.
    parser.add_argument(
        "--drones",
        type=int,
        default=1,
        help="Number of drones (launcher compatibility; must be 1 for solo)",
    )
    parser.add_argument("--drone-id", type=int, default=1)
    parser.add_argument("--udp-port", type=int, default=14551)
    parser.add_argument("--heartbeat-timeout", type=float, default=12.0)
    parser.add_argument("--takeoff-alt", type=float, default=TAKEOFF_ALT_M)
    args = parser.parse_args()

    if int(args.drones) != 1:
        raise SystemExit(f"solo scenario supports only --drones 1 (got {args.drones})")

    cfg = {"id": int(args.drone_id), "udp_port": int(args.udp_port), "role": "solo"}
    c = DroneController(cfg, logging_enabled=False)
    try:
        logger.info(
            "Connecting and initializing (heartbeat timeout %.1fs)...",
            float(args.heartbeat_timeout),
        )
        initialize_drone_solo(
            c,
            init_steps=INIT_STEPS,
            heartbeat_timeout_s=float(args.heartbeat_timeout),
            position_timeout_s=10.0,
        )
        logger.info("Connected: heartbeat + position stream OK. Waiting takeoff...")
        ok = _wait_for_takeoff(c, target_alt_m=float(args.takeoff_alt))
        if ok:
            logger.info("Takeoff confirmed: ~%.1fm reached.", float(args.takeoff_alt))
        else:
            logger.warning("Takeoff not confirmed, but init sequence finished.")
        time.sleep(1.0)
    finally:
        try:
            c.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()


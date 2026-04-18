#!/usr/bin/env python3
"""
Publish drone positions to the 2D visualizer over UDP.

Scenarios call publish_positions() from the coordinate exchange loop.
Format: positions = {drone_id: {"x": float, "y": float, "z": float}, ...}
Failures are suppressed so the simulation continues if the visualizer is not running.
"""
import json
import socket
from typing import Any, Dict, Optional

DEFAULT_VISUALIZER_PORT = 15551


def publish_positions(
    positions: Dict[Any, Dict[str, float]],
    host: str = "127.0.0.1",
    port: int = DEFAULT_VISUALIZER_PORT,
    rates: Optional[Dict[str, Optional[float]]] = None,
) -> None:
    """Send current drone positions to the visualizer via UDP.

    Args:
        positions: Dict mapping drone_id to {"x", "y", "z"} (NED, meters).
        host: Visualizer host (default 127.0.0.1).
        port: Visualizer UDP port (default 15551).
        rates: Optional {"follower_hz", "exchange_hz", "webots_step_hz"} for display.

    Returns:
        None.

    Note:
        Does not raise; if the visualizer is not running, packets are dropped silently.
    """
    if not positions and not rates:
        return
    try:
        payload_dict: Dict[str, Any] = {}
        if positions:
            payload_dict["positions"] = {
                str(k): {"x": v.get("x", 0), "y": v.get("y", 0), "z": v.get("z", 0)}
                for k, v in positions.items()
            }
        if rates is not None:
            payload_dict["rates"] = {
                "follower_hz": rates.get("follower_hz"),
                "exchange_hz": rates.get("exchange_hz"),
                "webots_step_hz": rates.get("webots_step_hz"),
            }
        payload = json.dumps(payload_dict).encode("utf-8")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(payload, (host, port))
        finally:
            sock.close()
    except (OSError, TypeError, ValueError):
        pass

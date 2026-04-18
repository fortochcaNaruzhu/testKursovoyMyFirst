"""
MAVLink protocol utilities: RC override, stream rate requests, constants.
"""

from typing import Any, Optional

from pymavlink import mavutil

# Neutral PWM value for RC channels (roll, pitch, throttle, yaw).
RC_NEUTRAL: int = 1500


def send_rc_override(
    drone: Any,
    chan1: int,
    chan2: int,
    chan3: int,
    chan4: int,
    controller: Optional[Any] = None,
) -> None:
    """
    Send RC_OVERRIDE command to the drone (channels 1–4: roll, pitch, throttle, yaw).

    Args:
        drone: MAVLink connection (mavutil.mavlink_connection).
        chan1: Roll / aileron (left-right).
        chan2: Pitch / elevator (forward-back).
        chan3: Throttle.
        chan4: Yaw / rudder (rotation).
        controller: Optional DroneController instance; if provided, updates
            last_rc_channels under controller.rc_channels_lock for keepalive.
    """
    drone.mav.rc_channels_override_send(
        drone.target_system,
        drone.target_component,
        chan1,
        chan2,
        chan3,
        chan4,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    if controller is not None and hasattr(controller, "rc_channels_lock"):
        with controller.rc_channels_lock:
            controller.last_rc_channels["roll"] = chan1
            controller.last_rc_channels["pitch"] = chan2
            controller.last_rc_channels["throttle"] = chan3
            controller.last_rc_channels["yaw"] = chan4


def request_position_stream_rate(master: Any, hz: int = 50) -> None:
    """
    Request LOCAL_POSITION_NED stream at the given rate.

    Args:
        master: MAVLink connection (mavutil.mavlink_connection).
        hz: Desired frequency in Hz (default 50).
    """
    interval_us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def request_attitude_stream_rate(master: Any, hz: int = 50) -> None:
    """
    Request ATTITUDE stream (roll, pitch, yaw in radians) at the given rate.

    Args:
        master: MAVLink connection (mavutil.mavlink_connection).
        hz: Desired frequency in Hz (default 50).
    """
    interval_us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )

"""
Geodetic helpers: SIM_STATE lat/lon and HOME_POSITION to local NED metres.

NED origin at home: x North, y East, z Down (m). Flat-earth approximation
is sufficient for typical SITL radii (cm–km).
"""

import math
from typing import Any, Tuple

# Mean Earth radius (m); close to WGS84 mean for local offsets.
_EARTH_RADIUS_M: float = 6371008.8


def sim_state_lat_lon_deg(msg: Any) -> Tuple[float, float]:
    """
    Read SIM_STATE latitude/longitude in degrees.

    Prefer ``lat_int`` / ``lon_int`` (degE7) when non-zero (MAVLink spec);
    otherwise use float ``lat`` / ``lon``.
    """
    lat_int = int(getattr(msg, "lat_int", 0) or 0)
    lon_int = int(getattr(msg, "lon_int", 0) or 0)
    lat = (lat_int / 1.0e7) if lat_int != 0 else float(getattr(msg, "lat", 0.0))
    lon = (lon_int / 1.0e7) if lon_int != 0 else float(getattr(msg, "lon", 0.0))
    return lat, lon


def home_position_lat_lon_alt_m(msg: Any) -> Tuple[float, float, float]:
    """
    Parse HOME_POSITION: latitude/longitude (deg), altitude MSL (m).

    MAVLink encodes latitude/longitude as int32 degE7 and altitude as int32 mm.
    """
    lat_deg = float(getattr(msg, "latitude", 0)) / 1.0e7
    lon_deg = float(getattr(msg, "longitude", 0)) / 1.0e7
    alt_raw = getattr(msg, "altitude", 0)
    if isinstance(alt_raw, float):
        # Some stacks expose altitude in metres as float.
        alt_m = float(alt_raw)
    else:
        try:
            alt_mm = int(alt_raw)
        except (TypeError, ValueError):
            alt_mm = 0
        alt_m = alt_mm / 1000.0
    return lat_deg, lon_deg, alt_m


def ned_metres_from_home(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    home_lat_deg: float,
    home_lon_deg: float,
    home_alt_m: float,
) -> Tuple[float, float, float]:
    """
    NED offset from home (small-angle flat-earth).

    Args:
        lat_deg: Vehicle latitude (deg).
        lon_deg: Vehicle longitude (deg).
        alt_m: Vehicle altitude MSL (m), same datum as home.
        home_lat_deg: Home latitude (deg).
        home_lon_deg: Home longitude (deg).
        home_alt_m: Home altitude MSL (m).

    Returns:
        ``(x, y, z)`` with x North, y East, z Down (m).
    """
    dlat = math.radians(lat_deg - home_lat_deg)
    dlon = math.radians(lon_deg - home_lon_deg)
    lat_h = math.radians(home_lat_deg)
    x = _EARTH_RADIUS_M * dlat
    y = _EARTH_RADIUS_M * dlon * math.cos(lat_h)
    z = home_alt_m - alt_m
    return x, y, z

"""Unit tests for SIM_STATE / HOME NED conversion helpers."""

import math
import unittest

from core.mavlink.geo_ned import ned_metres_from_home, sim_state_lat_lon_deg


class _FakeSim:
    def __init__(self) -> None:
        self.lat_int = int(47_1234567)  # ~47.1234567 deg
        self.lon_int = int(8_7654321)
        self.lat = 0.0
        self.lon = 0.0


class GeoNedTest(unittest.TestCase):
    def test_sim_state_prefers_lat_lon_int(self) -> None:
        msg = _FakeSim()
        lat, lon = sim_state_lat_lon_deg(msg)
        self.assertAlmostEqual(lat, 47.1234567, places=6)
        self.assertAlmostEqual(lon, 8.7654321, places=6)

    def test_ned_metres_from_home_small_offset(self) -> None:
        home_lat, home_lon, home_alt = 47.0, 8.0, 100.0
        lat, lon, alt = 47.001, 8.0, 100.0
        x, y, z = ned_metres_from_home(lat, lon, alt, home_lat, home_lon, home_alt)
        self.assertGreater(x, 100.0)
        self.assertLess(abs(y), 2.0)
        self.assertLess(abs(z), 1e-6)
        self.assertLess(x, 120.0)

    def test_ned_z_down_when_higher(self) -> None:
        x, y, z = ned_metres_from_home(0.0, 0.0, 200.0, 0.0, 0.0, 100.0)
        self.assertLess(z, -1e-6)
        self.assertTrue(math.isclose(z, -100.0, rel_tol=0.0, abs_tol=1e-6))


if __name__ == "__main__":
    unittest.main()

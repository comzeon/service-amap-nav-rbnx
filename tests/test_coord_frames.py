# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import unittest

from amap_nav_service import coord_frames as cf


class TestGcj02RoundTrip(unittest.TestCase):
    def test_roundtrip_error_sub_metre(self):
        for lon, lat in [(116.397428, 39.90923), (121.4737, 31.2304),
                         (114.057868, 22.543099)]:
            w = cf.gcj02_to_wgs84(lon, lat)
            back = cf.wgs84_to_gcj02(*w)
            err = cf.haversine_m(lon, lat, back[0], back[1])
            self.assertLess(err, 1.0, f"roundtrip err {err:.3f}m @ {lon},{lat}")

    def test_offset_is_large_not_identity(self):
        w = cf.gcj02_to_wgs84(116.397428, 39.90923)
        d = cf.haversine_m(116.397428, 39.90923, *w)
        self.assertGreater(d, 100.0)

    def test_out_of_china_identity(self):
        self.assertEqual(cf.wgs84_to_gcj02(-73.9857, 40.7484), (-73.9857, 40.7484))

    def test_utm_roundtrip(self):
        x, y, z = cf.to_utm(116.397428, 39.90923)
        lon, lat = cf.from_utm(x, y, z)
        self.assertLess(cf.haversine_m(116.397428, 39.90923, lon, lat), 1.0)


if __name__ == "__main__":
    unittest.main()

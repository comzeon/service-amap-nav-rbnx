# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import unittest
from unittest import mock

from amap_nav_service import amap_client


def _walk_payload() -> dict:
    return {
        "status": "1", "info": "OK", "infocode": "10000", "count": "1",
        "route": {
            "origin": "116.397428,39.90923",
            "destination": "116.427104,39.905152",
            "paths": [{
                "distance": "4379",
                "cost": {"duration": "3503"},
                "steps": [{
                    "instruction": "向北步行391米右转",
                    "orientation": "北", "road_name": "",
                    "step_distance": "391",
                    "cost": {"duration": "313"},
                    "navi": {"action": "右转", "assistant_action": "", "walk_type": "0"},
                    "polyline": "116.397435,39.909227;116.397387,39.910373;116.397348,39.911211",
                }, {
                    "instruction": "沿北京站东街步行11米到达目的地",
                    "orientation": "", "road_name": "北京站东街",
                    "step_distance": "11",
                    "cost": {"duration": "9"},
                    "navi": {"action": "到达", "assistant_action": "", "walk_type": "3"},
                    "polyline": "116.42724,39.905104",
                }],
            }],
        },
    }


class TestWalkingRoute(unittest.TestCase):
    @mock.patch.object(amap_client, "_get")
    def test_parses_steps_and_polyline(self, mock_get):
        mock_get.return_value = _walk_payload()
        route = amap_client.walking_route((116.397428, 39.90923), (116.427104, 39.905152))
        self.assertEqual(route.distance_m, 4379)
        self.assertEqual(route.duration_s, 3503)
        self.assertEqual(len(route.steps), 2)
        self.assertEqual(route.steps[0].walk_type, 0)
        self.assertEqual(route.steps[1].walk_type, 3)
        self.assertEqual(len(route.steps[0].polyline), 3)
        self.assertEqual(route.steps[0].polyline[0], (116.397435, 39.909227))
        args, _ = mock_get.call_args
        self.assertEqual(args[1]["show_fields"], "cost,navi,polyline")

    @mock.patch.object(amap_client, "_get")
    def test_raises_on_status_zero(self, mock_get):
        mock_get.side_effect = amap_client.AmapError("10009", "USERKEY_PLAT_NOMATCH")
        with self.assertRaises(amap_client.AmapError):
            amap_client.walking_route((1, 2), (3, 4))

    def test_missing_key_raises(self):
        with mock.patch.dict("os.environ", {}, clear=True):
            with self.assertRaises(amap_client.AmapError) as ctx:
                amap_client.walking_route((1, 2), (3, 4))
            self.assertEqual(ctx.exception.infocode, "NOKEY")

    @mock.patch.object(amap_client, "_get")
    def test_parse_polyline_skips_bad_pairs(self, mock_get):
        self.assertEqual(amap_client._parse_polyline("1.0,2.0;bad;3,4"), [(1.0, 2.0), (3.0, 4.0)])


class TestConvert(unittest.TestCase):
    @mock.patch.object(amap_client, "_get")
    def test_gps_to_gcj02(self, mock_get):
        mock_get.return_value = {"status": "1", "info": "ok",
                                 "locations": "116.403671,39.910633"}
        out = amap_client.gps_to_gcj02([(116.397428, 39.90923)])
        self.assertAlmostEqual(out[0][0], 116.403671, places=5)
        args, _ = mock_get.call_args
        self.assertEqual(args[1]["coordsys"], "gps")


if __name__ == "__main__":
    unittest.main()

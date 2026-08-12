# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import unittest

from amap_nav_service.amap_client import RouteStep, WalkingRoute
from amap_nav_service import route_planner as rp


def _step(wt: int, pts: list[tuple[float, float]], dist: int = 10,
          instr: str = "x") -> RouteStep:
    return RouteStep(instruction=instr, road_name="", distance_m=dist,
                     polyline=pts, walk_type=wt)


class TestPlan(unittest.TestCase):
    def test_normal_route_produces_waypoints(self):
        route = WalkingRoute(distance_m=100, duration_s=60, steps=[
            _step(0, [(116.0, 39.0), (116.001, 39.0), (116.002, 39.0)], 100),
        ])
        wps = rp.plan(route)
        self.assertTrue(wps)
        self.assertTrue(all(w.seg_type == "normal" for w in wps))

    def test_cross_road_marked(self):
        route = WalkingRoute(distance_m=20, duration_s=30, steps=[
            _step(0, [(116.0, 39.0)], 10),
            _step(1, [(116.0, 39.0), (116.001, 39.0)], 10),
        ])
        wps = rp.plan(route)
        self.assertTrue(any(w.seg_type == "cross_road" for w in wps))

    def test_blocked_walk_type_raises(self):
        route = WalkingRoute(distance_m=10, duration_s=10, steps=[
            _step(8, [(116.0, 39.0)], 10),  # 扶梯
        ])
        with self.assertRaises(rp.RouteNotTraversable):
            rp.plan(route)

    def test_unknown_walk_type_raises(self):
        # 严格白名单: 高德新增类型默认拒绝, 宁停勿闯
        route = WalkingRoute(distance_m=10, duration_s=10, steps=[
            _step(99, [(116.0, 39.0)], 10),
        ])
        with self.assertRaises(rp.RouteNotTraversable):
            rp.plan(route)

    def test_stairs_and_tunnel_mapped(self):
        route = WalkingRoute(distance_m=30, duration_s=30, steps=[
            _step(20, [(116.0, 39.0), (116.001, 39.0)], 15),
            _step(3, [(116.001, 39.0), (116.002, 39.0)], 15),
        ])
        wps = rp.plan(route)
        types = {w.seg_type for w in wps}
        self.assertIn("stairs", types)
        self.assertIn("tunnel", types)

    def test_no_geometry_raises(self):
        route = WalkingRoute(distance_m=10, duration_s=10, steps=[
            _step(0, [], 10),
        ])
        with self.assertRaises(rp.RouteNotTraversable):
            rp.plan(route)

    def test_simplify_spacing_within_max(self):
        pts = [(116.0 + i * 1e-5, 39.0) for i in range(100)]
        out = rp._simplify(pts, max_seg_m=40.0)
        self.assertLessEqual(len(out), len(pts))
        # acc 先加后判, 允许一步过冲: 上界 = max_seg + 单点间距(~0.9m) + 容差
        self.assertTrue(all(rp._seg_dist(out[i], out[i + 1]) <= 40.0 + 1.5
                            for i in range(len(out) - 1)))

    def test_simplify_keeps_corners(self):
        # 折线: 向东 30m 然后向北 30m —— 拐点必须保留
        pts = [(116.0, 39.0), (116.0003, 39.0), (116.0003, 39.0003)]
        out = rp._simplify(pts, max_seg_m=40.0)
        self.assertIn((116.0003, 39.0), out)  # 拐点


class TestSegDist(unittest.TestCase):
    def test_known_distance(self):
        d = rp._seg_dist((116.0, 39.0), (116.0, 39.001))
        self.assertAlmostEqual(d, 111.32, delta=1.0)


if __name__ == "__main__":
    unittest.main()

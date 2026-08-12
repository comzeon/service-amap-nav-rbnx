# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import unittest

from amap_nav_service import executor as ex


class TestCrossTrack(unittest.TestCase):
    def test_on_line_zero(self):
        d = ex.cross_track_m((0.0, 0.0), (0.0, 0.0), (100.0, 0.0))
        self.assertAlmostEqual(d, 0.0, places=6)

    def test_off_line_ten_metres(self):
        d = ex.cross_track_m((50.0, 10.0), (0.0, 0.0), (100.0, 0.0))
        self.assertAlmostEqual(d, 10.0, places=6)

    def test_beyond_segment_uses_endpoint(self):
        d = ex.cross_track_m((120.0, 0.0), (0.0, 0.0), (100.0, 0.0))
        self.assertAlmostEqual(d, 20.0, places=6)

    def test_zero_length_segment(self):
        d = ex.cross_track_m((3.0, 4.0), (0.0, 0.0), (0.0, 0.0))
        self.assertAlmostEqual(d, 5.0, places=6)


class TestExecutor(unittest.TestCase):
    def setUp(self):
        class FakeNav:
            def __init__(self):
                self.goals = []

            def go(self, x, y):
                self.goals.append((x, y))
                return f"run-{len(self.goals)}"

            def status(self):
                return "SUCCEEDED"

        self.fake = FakeNav()
        self.exe = ex.Executor(navigate=self.fake, cross_track_threshold_m=20.0)

    def test_dispatches_normal_then_gates_cross_road(self):
        self.exe.set_waypoints([(0.0, 0.0, "normal"), (10.0, 0.0, "cross_road")])
        self.exe.step_once()  # normal 段下发
        self.assertEqual(len(self.fake.goals), 1)
        self.assertEqual(self.exe.state, ex.st.EXECUTING)

        # 下一个 tick: cross_road 段应进闸门而非下发
        self.exe.step_once()
        self.assertEqual(self.exe.state, ex.st.CROSSING_WAIT)
        self.assertEqual(len(self.fake.goals), 1)  # 未下发
        self.assertFalse(self.exe.crossing_confirmed)

        # 未确认再 tick: 仍不发
        self.exe.step_once()
        self.assertEqual(len(self.fake.goals), 1)

        # 人工确认后下发
        self.exe.confirm_crossing(True)
        self.exe.step_once()
        self.assertEqual(len(self.fake.goals), 2)
        self.assertEqual(self.fake.goals[1], (10.0, 0.0))

    def test_crossing_denied_fails_task(self):
        self.exe.set_waypoints([(10.0, 0.0, "cross_road")])
        self.exe.step_once()  # 进闸门
        ok = self.exe.confirm_crossing(False)
        self.assertTrue(ok)
        self.assertEqual(self.exe.state, ex.st.FAILED)

    def test_crossing_confirm_when_not_waiting_rejected(self):
        self.exe.set_waypoints([(0.0, 0.0, "normal")])
        self.assertFalse(self.exe.confirm_crossing(True))

    def test_crossing_gate_timeout_fails(self):
        import time as _t
        self.exe.set_waypoints([(10.0, 0.0, "cross_road")])
        self.exe.step_once()  # 进闸门
        self.exe.crossing_wait_since = _t.time() - ex.CROSSING_WAIT_TIMEOUT_S - 1
        self.exe.step_once()
        self.assertEqual(self.exe.state, ex.st.FAILED)
        self.assertIn("timeout", self.exe.latest_detail)

    def test_consecutive_crossings_each_gated(self):
        # 连续两个过街段: 每段都需要独立人工确认
        self.exe.set_waypoints([
            (0.0, 0.0, "cross_road"),
            (50.0, 0.0, "cross_road"),
        ])
        self.exe.step_once()  # 段1 进闸门
        self.exe.confirm_crossing(True)
        self.exe.step_once()  # 段1 下发
        self.assertEqual(len(self.fake.goals), 1)
        self.exe.step_once()  # 段2 重新进闸门
        self.assertEqual(self.exe.state, ex.st.CROSSING_WAIT)
        self.assertEqual(len(self.fake.goals), 1)  # 未下发
        self.exe.confirm_crossing(True)
        self.exe.step_once()  # 段2 下发
        self.assertEqual(len(self.fake.goals), 2)

    def test_go_exception_fails_task(self):
        class BrokenNav:
            def go(self, x, y):
                raise RuntimeError("nav down")

        exe = ex.Executor(navigate=BrokenNav())
        exe.set_waypoints([(0.0, 0.0, "normal")])
        exe.step_once()
        self.assertEqual(exe.state, ex.st.FAILED)
        self.assertIn("nav down", exe.latest_detail)

    def test_off_track_triggers_replanning(self):
        replanned = []
        self.exe.on_replan_requested = lambda: replanned.append(True)
        self.exe.set_waypoints([(0.0, 0.0, "normal"), (100.0, 0.0, "normal")])
        self.exe.step_once()  # idx=1, pose 未知, 无偏离检查
        self.exe.update_pose(50.0, 30.0)  # 偏离 30m > 20m
        self.exe.step_once()
        self.assertEqual(self.exe.state, ex.st.REPLANNING)
        self.assertEqual(replanned, [True])

    def test_all_dispatched_arrives(self):
        self.exe.set_waypoints([(0.0, 0.0, "normal"), (1.0, 0.0, "normal")])
        self.exe.step_once()
        self.exe.step_once()
        self.exe.step_once()  # 已发完
        self.assertEqual(self.exe.state, ex.st.ARRIVED)


if __name__ == "__main__":
    unittest.main()

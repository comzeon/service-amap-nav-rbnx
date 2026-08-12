# SPDX-License-Identifier: MulanPSL-2.0
"""service 层测试 — 用 sys.modules 桩掉部署环境依赖 (navigation_mcp/robonix_api).

覆盖审查 #5 指出的盲区: amap_goal 各失败分支、中能量门、取消短路、状态回写。
"""
from __future__ import annotations

import sys
import types
import unittest
from unittest import mock

# ── 部署环境依赖桩 (在 import service 之前) ────────────────────────────
class _StubMsg:
    """模拟 codegen dataclass: 支持 kwargs 构造 + 属性访问."""

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


nav_mcp = types.ModuleType("navigation_mcp")
for _n in ("AmapGoal_Request", "AmapGoal_Response",
           "AmapStatus_Request", "AmapStatus_Response",
           "AmapCancel_Request", "AmapCancel_Response",
           "AmapCrossingConfirm_Request", "AmapCrossingConfirm_Response"):
    setattr(nav_mcp, _n, type(_n, (_StubMsg,), {}))
sys.modules["navigation_mcp"] = nav_mcp


class _FakeService:
    def __init__(self, id, namespace):
        self.id = id
        self.namespace = namespace
        self.tools = {}
        self.init = None

    def mcp(self, name):
        def deco(fn):
            self.tools[name] = fn
            return fn
        return deco

    def on_init(self, fn):
        self.init = fn
        return fn

    def run(self):
        pass


robonix_api = types.ModuleType("robonix_api")
robonix_api.ATLAS = mock.MagicMock()
robonix_api.Service = _FakeService
robonix_api.Ok = object()
robonix_api.Err = type("Err", (), {"__init__": lambda self, m: setattr(self, "msg", m)})
sys.modules["robonix_api"] = robonix_api

from amap_nav_service import service as svc  # noqa: E402
from amap_nav_service import state as st  # noqa: E402


def _req(cls, **kw):
    r = cls()
    for k, v in kw.items():
        setattr(r, k, v)
    return r


class TestAmapGoal(unittest.TestCase):
    def setUp(self):
        # 干净的全局状态
        svc._TASKS.clear()
        svc._LATEST_TASK_ID = None
        svc._UTM_ZONE = None
        svc._EXECUTOR = mock.MagicMock()
        svc._EXECUTOR.state = st.IDLE
        self._resp = nav_mcp.AmapGoal_Response
        self.env = mock.patch.dict("os.environ", {
            "AMAP_WEB_KEY": "k",
            "AMAP_ORIGIN_WGS84": "116.39,39.90",
        })
        self.env.start()

    def tearDown(self):
        self.env.stop()

    def test_not_initialized(self):
        svc._EXECUTOR = None
        r = svc.amap_goal(_req(nav_mcp.AmapGoal_Request,
                               dest_gcj02="116.42,39.90", dest_address="",
                               confirm_medium_energy=True))
        self.assertFalse(r.accepted)

    def test_bad_dest(self):
        for bad in ("abc", "999,999", "116.42"):
            r = svc.amap_goal(_req(nav_mcp.AmapGoal_Request,
                                   dest_gcj02=bad, dest_address="",
                                   confirm_medium_energy=True))
            self.assertFalse(r.accepted, f"should reject {bad!r}")
            self.assertIn("bad dest_gcj02", r.detail)

    def test_no_gps(self):
        with mock.patch.dict("os.environ", {}, clear=True):
            r = svc.amap_goal(_req(nav_mcp.AmapGoal_Request,
                                   dest_gcj02="116.42,39.90", dest_address="",
                                   confirm_medium_energy=True))
        self.assertFalse(r.accepted)
        self.assertIn("GPS fix", r.detail)

    @mock.patch.object(svc, "gps_to_gcj02", return_value=[(116.40, 39.91)])
    def test_distance_cap(self, mock_gps):
        # 北京起点 → 上海终点 远超 20km 护栏
        r = svc.amap_goal(_req(nav_mcp.AmapGoal_Request,
                               dest_gcj02="121.47,31.23", dest_address="",
                               confirm_medium_energy=True))
        self.assertFalse(r.accepted)
        self.assertIn("cap", r.detail)

    @mock.patch.object(svc, "walking_route")
    @mock.patch.object(svc, "gps_to_gcj02", return_value=[(116.40, 39.91)])
    def test_medium_energy_gate(self, mock_gps, mock_walk):
        from amap_nav_service.amap_client import RouteStep, WalkingRoute
        mock_walk.return_value = WalkingRoute(distance_m=50, duration_s=30, steps=[
            RouteStep(instruction="过街", road_name="", distance_m=10,
                      polyline=[(116.40, 39.91), (116.401, 39.91)], walk_type=1),
        ])
        # 未确认 → 拒绝
        r = svc.amap_goal(_req(nav_mcp.AmapGoal_Request,
                               dest_gcj02="116.401,39.91", dest_address="",
                               confirm_medium_energy=False))
        self.assertFalse(r.accepted)
        self.assertIn("medium-energy", r.detail)
        # 确认 → 接受
        r = svc.amap_goal(_req(nav_mcp.AmapGoal_Request,
                               dest_gcj02="116.401,39.91", dest_address="",
                               confirm_medium_energy=True))
        self.assertTrue(r.accepted)
        self.assertEqual(svc._LATEST_TASK_ID, r.task_id)

    @mock.patch.object(svc, "walking_route")
    @mock.patch.object(svc, "gps_to_gcj02", return_value=[(116.40, 39.91)])
    def test_success_path_registers_task(self, mock_gps, mock_walk):
        from amap_nav_service.amap_client import RouteStep, WalkingRoute
        mock_walk.return_value = WalkingRoute(distance_m=100, duration_s=60, steps=[
            RouteStep(instruction="直行", road_name="", distance_m=100,
                      polyline=[(116.40, 39.91), (116.401, 39.91)], walk_type=0),
        ])
        r = svc.amap_goal(_req(nav_mcp.AmapGoal_Request,
                               dest_gcj02="116.401,39.91", dest_address="",
                               confirm_medium_energy=True))
        self.assertTrue(r.accepted)
        self.assertIn(r.task_id, svc._TASKS)
        svc._EXECUTOR.set_waypoints.assert_called_once()


class TestCancel(unittest.TestCase):
    def setUp(self):
        svc._TASKS.clear()
        svc._LATEST_TASK_ID = "t1"
        svc._TASKS["t1"] = st.Task(task_id="t1", dest_gcj02=(116.0, 39.0),
                                   state=st.EXECUTING)
        svc._EXECUTOR = mock.MagicMock()
        svc._EXECUTOR.state = st.EXECUTING

    def test_cancel_stops_executor(self):
        r = svc.amap_cancel(_req(nav_mcp.AmapCancel_Request, task_id="t1"))
        self.assertTrue(r.accepted)
        self.assertEqual(svc._EXECUTOR.state, st.CANCELLED)
        self.assertEqual(svc._TASKS["t1"].state, st.CANCELLED)

    def test_cancel_unknown_task(self):
        r = svc.amap_cancel(_req(nav_mcp.AmapCancel_Request, task_id="nope"))
        self.assertFalse(r.accepted)


class TestSyncTaskState(unittest.TestCase):
    def test_arrived_maps_to_done(self):
        svc._TASKS.clear()
        svc._LATEST_TASK_ID = "t1"
        task = st.Task(task_id="t1", dest_gcj02=(116.0, 39.0), state=st.EXECUTING)
        svc._TASKS["t1"] = task
        svc._EXECUTOR = mock.MagicMock()
        svc._EXECUTOR.state = st.ARRIVED
        svc._EXECUTOR.latest_detail = "all waypoints dispatched"
        svc._sync_task_state()
        self.assertEqual(task.state, st.DONE)

    def test_failed_maps_to_failed(self):
        svc._TASKS.clear()
        svc._LATEST_TASK_ID = "t1"
        task = st.Task(task_id="t1", dest_gcj02=(116.0, 39.0), state=st.EXECUTING)
        svc._TASKS["t1"] = task
        svc._EXECUTOR = mock.MagicMock()
        svc._EXECUTOR.state = st.FAILED
        svc._EXECUTOR.latest_detail = "crossing denied"
        svc._sync_task_state()
        self.assertEqual(task.state, st.FAILED)


class TestCrossingConfirm(unittest.TestCase):
    def test_not_waiting_rejected(self):
        svc._EXECUTOR = mock.MagicMock()
        svc._EXECUTOR.confirm_crossing.return_value = False
        r = svc.amap_crossing_confirm(_req(nav_mcp.AmapCrossingConfirm_Request,
                                           task_id="", proceed=True))
        self.assertFalse(r.accepted)


if __name__ == "__main__":
    unittest.main()

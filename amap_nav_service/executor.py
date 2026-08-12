# SPDX-License-Identifier: MulanPSL-2.0
"""executor — 逐段执行 + 过街闸门 + 偏离检测 (设计 4.3).

navigate 抽象: 传入实现了 go(x, y) -> run_id 与 status() -> str 的客户端
（生产=ATLAS 解析后的 navigate MCP/ROS 调用; 测试=fake）。
纯函数 cross_track_m 可单测; Executor.step_once 由外部 1Hz ticker 驱动。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

from . import state as st

CROSSING_WAIT_TIMEOUT_S = 90.0
ARRIVE_M = 1.5
REPLAN_THRESHOLD_M = 20.0  # 默认 20m (设计 4.3)


@dataclass
class WaypointUTM:
    x: float
    y: float
    seg_type: str = "normal"


def cross_track_m(pose: tuple[float, float],
                  seg_start: tuple[float, float],
                  seg_end: tuple[float, float]) -> float:
    """P 到线段 S-E 的横向距离（米）. 投影落在段外时取到端点距离."""
    px, py = pose
    sx, sy = seg_start
    ex_, ey = seg_end
    dx, dy = ex_ - sx, ey - sy
    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-9:
        return math.hypot(px - sx, py - sy)
    t = ((px - sx) * dx + (py - sy) * dy) / seg_len2
    t = max(0.0, min(1.0, t))
    cx, cy = sx + t * dx, sy + t * dy
    return math.hypot(px - cx, py - cy)


class Executor:
    def __init__(self, navigate, cross_track_threshold_m: float = REPLAN_THRESHOLD_M):
        self.navigate = navigate
        self.cross_track_threshold_m = cross_track_threshold_m
        self.waypoints: list[WaypointUTM] = []
        self.idx = 0
        self.state = st.IDLE
        self.crossing_confirmed = False
        self.crossing_wait_since: float | None = None
        self.pose_utm: tuple[float, float] | None = None
        self.latest_detail = ""
        self.on_replan_requested: callable | None = None  # 置位→上层触发重规划

    def set_waypoints(self, wps: list[tuple[float, float, str]]) -> None:
        self.waypoints = [WaypointUTM(x, y, t) for x, y, t in wps]
        self.idx = 0
        self.state = st.EXECUTING
        self.crossing_confirmed = False
        self.crossing_wait_since = None
        self.latest_detail = f"loaded {len(wps)} waypoints"

    def update_pose(self, x: float, y: float) -> None:
        self.pose_utm = (x, y)

    def step_once(self) -> None:
        """一次 tick（sim 里 1Hz 驱动）."""
        if self.state not in (st.EXECUTING, st.CROSSING_WAIT):
            return
        if self.idx >= len(self.waypoints):
            self.state = st.ARRIVED
            self.latest_detail = "all waypoints dispatched"
            return
        wp = self.waypoints[self.idx]

        if self.state == st.CROSSING_WAIT:
            self._tick_crossing_wait()
            return

        # 偏离检测: 与"当前段"（上一点→当前点）的横向距离
        if self.pose_utm is not None and self.idx > 0:
            prev = self.waypoints[self.idx - 1]
            off = cross_track_m(self.pose_utm, (prev.x, prev.y), (wp.x, wp.y))
            if off > self.cross_track_threshold_m:
                self.state = st.REPLANNING
                self.latest_detail = f"off-track {off:.1f}m > {self.cross_track_threshold_m}m"
                if self.on_replan_requested:
                    self.on_replan_requested()
                return

        # 过街段: 进闸门, 不发 goal
        if wp.seg_type == "cross_road" and not self.crossing_confirmed:
            self.state = st.CROSSING_WAIT
            self.crossing_wait_since = time.time()
            self.latest_detail = "cross_road gate: waiting human confirm"
            return

        # 发 goal 并推进
        run = self.navigate.go(wp.x, wp.y)
        self.latest_detail = f"seg {self.idx} -> ({wp.x:.1f},{wp.y:.1f}) run={run}"
        self.idx += 1
        self.crossing_confirmed = False  # 每段重置; 过街段由 confirm 置位

    def _tick_crossing_wait(self) -> None:
        if self.crossing_confirmed:
            self.state = st.EXECUTING
            self.crossing_wait_since = None
            return
        if self.crossing_wait_since is not None and \
                time.time() - self.crossing_wait_since > CROSSING_WAIT_TIMEOUT_S:
            self.state = st.FAILED
            self.latest_detail = "crossing gate timeout"

    def confirm_crossing(self, proceed: bool) -> bool:
        if self.state != st.CROSSING_WAIT:
            return False
        if not proceed:
            self.state = st.FAILED
            self.latest_detail = "crossing denied by operator"
            return True
        self.crossing_confirmed = True
        self.state = st.EXECUTING
        self.crossing_wait_since = None
        self.latest_detail = "crossing confirmed"
        return True

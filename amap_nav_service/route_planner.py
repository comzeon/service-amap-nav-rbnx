# SPDX-License-Identifier: MulanPSL-2.0
"""route_planner — 高德步行路线 → 可执行航点列表.

- walk_type 白名单（Lite3 能爬台阶; 排除扶梯/索道/轮渡等）
- cross_road 段标记（过街闸门用）
- 抽稀: 保留拐点, 相邻航点 ≤ MAX_SEG_M
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .amap_client import WalkingRoute

# 设计 4.1 白名单: 允许(含未知类型默认放行) / 明确排除
ALLOWED_WALK_TYPES = {0, 1, 3, 4, 6, 7, 20, 21, 22, 23}
BLOCKED_WALK_TYPES = {8, 9, 10, 11, 14, 15, 16, 30}
CROSS_ROAD_TYPE = 1
MAX_SEG_M = 40.0
MIN_KEEP_M = 5.0
TURN_DEG = 30.0


@dataclass
class RouteWaypoint:
    lon: float
    lat: float
    seg_type: str  # normal | cross_road | stairs | bridge | tunnel | tactile(P3)
    step_idx: int


class RouteNotTraversable(RuntimeError):
    """路线上存在白名单外路段."""


def _seg_type(walk_type: int) -> str:
    return {
        CROSS_ROAD_TYPE: "cross_road",
        20: "stairs",
        4: "bridge",
        22: "bridge",
        3: "tunnel",
        23: "tunnel",
    }.get(walk_type, "normal")


def _seg_dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    """等距圆柱近似距离（米）, ≤40m 尺度足够."""
    dlat = (b[1] - a[1]) * 111320.0
    dlon = (b[0] - a[0]) * 111320.0 * math.cos(math.radians((a[1] + b[1]) / 2))
    return math.hypot(dlon, dlat)


def _simplify(pts: list[tuple[float, float]], max_seg_m: float = MAX_SEG_M,
              min_keep_m: float = MIN_KEEP_M, turn_deg: float = TURN_DEG
              ) -> list[tuple[float, float]]:
    """贪心抽稀: 保留拐点 + 累计距离达 max_seg_m 时保点.

    间距保证 ≤ max_seg_m + 单段步长（acc 先加后判, 允许一步过冲）;
    min_keep_m 预留（防过密点抖动）, 当前实现不强制下界.
    """
    if len(pts) <= 2:
        return list(pts)
    out: list[tuple[float, float]] = [pts[0]]
    acc = 0.0
    prev_bearing: float | None = None
    for a, b in zip(pts, pts[1:]):
        d = _seg_dist(a, b)
        bearing = math.degrees(math.atan2(b[0] - a[0], b[1] - a[1]))
        turn = 0.0 if prev_bearing is None else abs((bearing - prev_bearing + 180) % 360 - 180)
        if turn > turn_deg:
            # 拐点: 发射 a（转角发生处）, 距离从 a 继续累积
            if out[-1] != a:
                out.append(a)
            acc = d
        else:
            acc += d
            if acc >= max_seg_m:
                out.append(b)
                acc = 0.0
        prev_bearing = bearing
    if out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def plan(route: WalkingRoute) -> list[RouteWaypoint]:
    """过滤 + 标记 + 抽稀. 白名单外路段抛 RouteNotTraversable."""
    wps: list[RouteWaypoint] = []
    for idx, step in enumerate(route.steps):
        if step.walk_type in BLOCKED_WALK_TYPES:
            raise RouteNotTraversable(
                f"step {idx} walk_type={step.walk_type} blocked: {step.instruction}")
        st = _seg_type(step.walk_type)
        if not step.polyline:
            continue  # 无几何, 跳过(防御)
        for lon, lat in _simplify(step.polyline):
            wps.append(RouteWaypoint(lon=lon, lat=lat, seg_type=st, step_idx=idx))
    if not wps:
        raise RouteNotTraversable("route has no executable geometry")
    return wps

#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""amap_nav service — MCP tools + executor wiring.

Consumes `robonix/service/navigation/navigate` (+ status/cancel) as client,
resolved through ATLAS at init. Exposes:
  robonix/service/navigation/amap_goal / amap_status / amap_cancel / amap_crossing_confirm

Phase 1: navigate 客户端用 _StubNav 占位（sim/真机接线见 Task 10/11）。
"""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
import uuid

from robonix_api import ATLAS, Service, Ok, Err

from . import coord_frames as cf
from . import executor as ex
from . import route_planner as rp
from . import state as st
from .amap_client import AmapError, walking_route, gps_to_gcj02

log = logging.getLogger("amap_nav")

amap_nav = Service(id="amap_nav", namespace="robonix/service/navigation")

_TASKS: dict[str, st.Task] = {}
_LATEST_TASK_ID: str | None = None
_EXECUTOR: ex.Executor | None = None
_TICKER: threading.Thread | None = None
_TICK_EVT = threading.Event()
_LOCK = threading.Lock()
_UTM_ZONE: int | None = None
_POSE_FILE: str | None = None

# 任务距离上限 (设计 4.5 护栏; config 可覆盖)
MAX_ROUTE_KM = float(os.environ.get("AMAP_MAX_ROUTE_KM", "20.0"))

# executor 状态 → Task 状态映射 (审查 #1/#10: 双状态域同步)
_EXECUTOR_STATE_MAP = {
    st.EXECUTING: st.EXECUTING,
    st.CROSSING_WAIT: st.CROSSING_WAIT,
    st.REPLANNING: st.REPLANNING,
    st.ARRIVED: st.DONE,
    st.FAILED: st.FAILED,
    st.CANCELLED: st.CANCELLED,
}


def _sync_task_state() -> None:
    """把 executor 状态回写到当前 Task (供 amap_status 可观测)."""
    task = _TASKS.get(_LATEST_TASK_ID or "") if _LATEST_TASK_ID else None
    if task is None or _EXECUTOR is None:
        return
    es = _EXECUTOR.state
    if es == st.IDLE:
        return
    ts = _EXECUTOR_STATE_MAP.get(es)
    if ts is None or ts == task.state:
        return
    try:
        task.transition(ts, _EXECUTOR.latest_detail or "")
    except ValueError:
        pass  # Task 已到终态, executor 迟到的状态回写忽略


def _resolve_nav_endpoint(deadline_s: float = 30.0) -> dict[str, str]:
    """解析 navigate 能力端点（参照 simple_nav.atlas_bridge.resolve_inputs）."""
    wanted = {
        "navigate": "robonix/service/navigation/navigate",
        "status": "robonix/service/navigation/navigate/status",
        "cancel": "robonix/service/navigation/navigate/cancel",
    }
    resolved: dict[str, str] = {}
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        for key, contract in wanted.items():
            if key in resolved:
                continue
            caps = ATLAS.find_capability(contract_id=contract, transport="ros2")
            if not caps:
                continue
            try:
                ch = amap_nav.connect_capability(caps[0], contract, "ros2")
            except Exception:  # noqa: BLE001
                continue
            ep = ch.endpoint
            ch.close()
            if ep:
                resolved[key] = ep
        if len(resolved) == len(wanted):
            break
        time.sleep(2.0)
    return resolved


def _current_origin_wgs84() -> tuple[float, float] | None:
    """Phase 1: 从 AMAP_ORIGIN_WGS84 环境变量取 WGS84 起点 (lon,lat).

    Sim (Task 10): fake GPS 节点写入该变量或订阅 /amap_gps 后写入。
    真机 (P1.7): 替换为 RTK 话题订阅。
    """
    v = os.environ.get("AMAP_ORIGIN_WGS84")
    if v:
        try:
            lon, lat = (float(x) for x in v.split(","))
            return lon, lat
        except ValueError:
            log.warning("bad AMAP_ORIGIN_WGS84: %r", v)
    return None


# ── MCP tools ─────────────────────────────────────────────────────────
# 类型由 rbnx codegen --mcp 从包内 capabilities/ 契约生成 (navigation_mcp)。
from navigation_mcp import (  # noqa: E402
    AmapGoal_Request, AmapGoal_Response,
    AmapStatus_Request, AmapStatus_Response,
    AmapCancel_Request, AmapCancel_Response,
    AmapCrossingConfirm_Request, AmapCrossingConfirm_Response,
)


@amap_nav.mcp("robonix/service/navigation/amap_goal")
def amap_goal(req: AmapGoal_Request) -> AmapGoal_Response:
    """开始一个高德辅助 A→B 任务. dest_gcj02 或 dest_address 二选一;
    confirm_medium_energy 必须为 True 才允许含过街/天桥段."""
    global _LATEST_TASK_ID, _UTM_ZONE
    if _EXECUTOR is None:
        return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                 detail="service not initialized")

    # 终点解析: GCJ02 优先; 地址走 maps_geo 由 pilot MCP 层转坐标(Phase 2)
    dest: tuple[float, float] | None = None
    if req.dest_gcj02.strip():
        try:
            lon, lat = (float(v) for v in req.dest_gcj02.split(","))
            if not (math.isfinite(lon) and math.isfinite(lat)
                    and 72.0 <= lon <= 138.0 and 0.8 <= lat <= 56.0):
                raise ValueError("out of range")
            dest = (lon, lat)
        except ValueError:
            return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                     detail=f"bad dest_gcj02: {req.dest_gcj02!r}")
    if dest is None:
        return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                 detail="dest_address not yet supported (use dest_gcj02)")

    # 起点: RTK/GPS → GCJ02
    origin_wgs = _current_origin_wgs84()
    if origin_wgs is None:
        return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                 detail="no GPS fix (AMAP_ORIGIN_WGS84 unset)")
    try:
        origin_gcj = gps_to_gcj02([origin_wgs])[0]
    except AmapError as e:
        return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                 detail=f"gps->gcj02 failed: {e}")

    # 距离护栏 (设计 4.5)
    if cf.haversine_m(origin_gcj[0], origin_gcj[1], dest[0], dest[1]) > MAX_ROUTE_KM * 1000:
        return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                 detail=f"task distance exceeds {MAX_ROUTE_KM:.0f}km cap")

    task_id = "amap-" + uuid.uuid4().hex[:8]
    try:
        route = walking_route(origin_gcj, dest)
        wps_gcj = rp.plan(route)
    except AmapError as e:
        return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                 detail=f"amap error: {e}")
    except rp.RouteNotTraversable as e:
        return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                 detail=f"route not traversable: {e}")

    # 中能量确认门 (设计 4.3): 含过街/天桥/台阶段必须用户确认
    medium = {w.seg_type for w in wps_gcj} & {"cross_road", "stairs", "bridge"}
    if medium and not req.confirm_medium_energy:
        return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                 detail=f"medium-energy segments present {sorted(medium)}; "
                                        f"set confirm_medium_energy=true")

    # 转 UTM 执行帧 (固定带号)
    try:
        wps_utm: list[tuple[float, float, str]] = []
        for w in wps_gcj:
            lon, lat = cf.gcj02_to_wgs84(w.lon, w.lat)
            x, y, z = cf.to_utm(lon, lat, _UTM_ZONE)
            _UTM_ZONE = z
            wps_utm.append((x, y, w.seg_type))
    except Exception as e:  # noqa: BLE001 — pyproj 缺失/投影失败
        return AmapGoal_Response(accepted=False, task_id="", route_summary="",
                                 detail=f"UTM conversion failed: {e}")

    task = st.Task(task_id=task_id, dest_gcj02=dest)
    with _LOCK:
        _TASKS[task_id] = task
        _LATEST_TASK_ID = task_id
    _EXECUTOR.set_waypoints(wps_utm)
    task.transition(st.EXECUTING, f"{len(wps_utm)} waypoints")
    summary = json.dumps({
        "distance_m": route.distance_m,
        "duration_s": route.duration_s,
        "waypoints": len(wps_utm),
        "seg_types": sorted({w.seg_type for w in wps_gcj}),
    }, ensure_ascii=False)
    log.info("amap_goal %s: %s", task_id, summary)
    return AmapGoal_Response(accepted=True, task_id=task_id,
                             route_summary=summary, detail="task started")


@amap_nav.mcp("robonix/service/navigation/amap_status")
def amap_status(req: AmapStatus_Request) -> AmapStatus_Response:
    task_id = req.task_id or _LATEST_TASK_ID or ""
    with _LOCK:
        task = _TASKS.get(task_id)
    if task is None:
        return AmapStatus_Response(known=False, state="", detail="no task", current_step="")
    step = ""
    if _EXECUTOR is not None and _EXECUTOR.idx < len(_EXECUTOR.waypoints):
        wp = _EXECUTOR.waypoints[_EXECUTOR.idx]
        step = json.dumps({"idx": _EXECUTOR.idx, "seg_type": wp.seg_type,
                           "detail": _EXECUTOR.latest_detail}, ensure_ascii=False)
    return AmapStatus_Response(known=True, state=task.state, detail=task.detail,
                               current_step=step)


@amap_nav.mcp("robonix/service/navigation/amap_cancel")
def amap_cancel(req: AmapCancel_Request) -> AmapCancel_Response:
    task_id = req.task_id or _LATEST_TASK_ID or ""
    with _LOCK:
        task = _TASKS.get(task_id)
        if task is not None and task.state not in st.TERMINAL:
            task.transition(st.CANCELLED, "cancelled by operator")
    if task is None:
        return AmapCancel_Response(accepted=False, detail="no task")
    # 关键: 停止 executor, 立即停止发新 goal (审查 #2: 取消必须停运动)
    if _EXECUTOR is not None:
        _EXECUTOR.cancel()
    return AmapCancel_Response(accepted=True, detail="cancelled")


@amap_nav.mcp("robonix/service/navigation/amap_crossing_confirm")
def amap_crossing_confirm(req: AmapCrossingConfirm_Request) -> AmapCrossingConfirm_Response:
    if _EXECUTOR is None:
        return AmapCrossingConfirm_Response(accepted=False, detail="not initialized")
    # 任务归属校验: 只允许确认最近任务 (或显式指定同一任务)
    task_id = req.task_id or _LATEST_TASK_ID or ""
    with _LOCK:
        task = _TASKS.get(task_id)
    if task is None or task.state != st.CROSSING_WAIT:
        return AmapCrossingConfirm_Response(accepted=False,
                                            detail="no task waiting at crossing gate")
    ok = _EXECUTOR.confirm_crossing(bool(req.proceed))
    return AmapCrossingConfirm_Response(accepted=ok,
                                        detail="confirmed" if ok else "no crossing gate active")


# ── ticker: 驱动 executor.step_once (1Hz) ─────────────────────────────
def _ticker_loop() -> None:
    while not _TICK_EVT.wait(1.0):
        if _EXECUTOR is None:
            continue
        # 可选位姿源: AMAP_POSE_FILE 每 tick 读 "x,y" (UTM), 供偏离检测
        if _POSE_FILE:
            try:
                with open(_POSE_FILE, "r", encoding="utf-8") as f:
                    x, y = (float(v) for v in f.read().strip().split(","))
                _EXECUTOR.update_pose(x, y)
            except Exception:  # noqa: BLE001 — 读不到/解析失败就跳过
                pass
        try:
            _EXECUTOR.step_once()
        except Exception:  # noqa: BLE001 — ticker 不能死
            log.exception("ticker step_once failed")
        _sync_task_state()


# ── lifecycle ─────────────────────────────────────────────────────────
@amap_nav.on_init
def init(cfg):
    global _EXECUTOR, _TICKER, MAX_ROUTE_KM, _POSE_FILE
    if not os.environ.get("AMAP_WEB_KEY"):
        return Err("AMAP_WEB_KEY not set")
    params = (cfg or {}).get("amap_nav_params") or {}
    MAX_ROUTE_KM = float(params.get("max_route_km", MAX_ROUTE_KM))
    cross_track = float(params.get("cross_track_threshold_m",
                                   ex.REPLAN_THRESHOLD_M))
    crossing_timeout = float(params.get("crossing_wait_timeout_s",
                                        ex.CROSSING_WAIT_TIMEOUT_S))
    _POSE_FILE = os.environ.get("AMAP_POSE_FILE")

    endpoints = _resolve_nav_endpoint()
    if "navigate" not in endpoints:
        return Err("missing navigate capability (is a nav service online?)")

    # Phase 1: stub; Task 10 sim 接线替换为真实 navigate MCP 调用
    class _StubNav:
        def go(self, x, y):
            return "stub-run"

    _EXECUTOR = ex.Executor(navigate=_StubNav(),
                            cross_track_threshold_m=cross_track,
                            crossing_wait_timeout_s=crossing_timeout)
    _TICKER = threading.Thread(target=_ticker_loop, name="amap-nav-ticker", daemon=True)
    _TICKER.start()
    log.info("amap_nav init ok: nav=%s cross_track=%.1f timeout=%.0f",
             endpoints, cross_track, crossing_timeout)
    return Ok()


def main() -> int:
    amap_nav.run()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())

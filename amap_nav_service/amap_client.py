# SPDX-License-Identifier: MulanPSL-2.0
"""amap_client — 高德 Web服务 API 薄封装（纯 HTTP, 无 ROS 依赖）.

- v5 步行路线规划: /v5/direction/walking?show_fields=cost,navi,polyline
  → 每步含 polyline 坐标点串(GCJ02) + navi.walk_type
- v3 坐标转换: /v3/assistant/coordinate/convert (gps→高德)
Key 从环境变量 AMAP_WEB_KEY 读取（部署时注入, 不落盘明文）。
"""
from __future__ import annotations

import json
import os
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass

BASE = "https://restapi.amap.com"
WALKING_V5 = "/v5/direction/walking"
CONVERT_V3 = "/v3/assistant/coordinate/convert"


class AmapError(RuntimeError):
    def __init__(self, infocode: str, info: str):
        super().__init__(f"AMap {infocode}: {info}")
        self.infocode = infocode
        self.info = info


def _get(path: str, params: dict, key: str | None = None,
         timeout: float = 8.0, retries: int = 2) -> dict:
    key = key or os.environ.get("AMAP_WEB_KEY", "")
    if not key:
        raise AmapError("NOKEY", "AMAP_WEB_KEY not set")
    url = BASE + path + "?" + urllib.parse.urlencode(dict(params, key=key))
    last: Exception | None = None
    for attempt in range(retries + 1):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r:
                data = json.loads(r.read().decode("utf-8"))
        except Exception as e:  # noqa: BLE001 — network/parse retry
            last = e
            time.sleep(0.5 * (attempt + 1))
            continue
        if data.get("status") != "1":
            raise AmapError(str(data.get("infocode", "?")), str(data.get("info", "?")))
        return data
    raise AmapError("NETWORK", f"after {retries + 1} attempts: {last}")


@dataclass
class RouteStep:
    instruction: str
    road_name: str
    distance_m: int
    polyline: list[tuple[float, float]]  # GCJ02
    walk_type: int


@dataclass
class WalkingRoute:
    distance_m: int
    duration_s: int
    steps: list[RouteStep]


def _parse_polyline(s: str) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for pair in s.split(";"):
        if "," not in pair:
            continue
        lon, lat = pair.split(",", 1)
        try:
            pts.append((float(lon), float(lat)))
        except ValueError:
            continue
    return pts


def walking_route(origin: tuple[float, float], destination: tuple[float, float],
                  key: str | None = None) -> WalkingRoute:
    """v5 步行路线规划（100km 内）. 输入输出均为 GCJ02."""
    data = _get(WALKING_V5, {
        "origin": f"{origin[0]:.6f},{origin[1]:.6f}",
        "destination": f"{destination[0]:.6f},{destination[1]:.6f}",
        "show_fields": "cost,navi,polyline",
    }, key=key)
    path = data["route"]["paths"][0]
    steps: list[RouteStep] = []
    for st in path.get("steps", []):
        navi = st.get("navi", {}) or {}
        wt = str(navi.get("walk_type", "0"))
        steps.append(RouteStep(
            instruction=st.get("instruction", ""),
            road_name=st.get("road_name", ""),
            distance_m=int(st.get("step_distance", 0) or 0),
            polyline=_parse_polyline(st.get("polyline", "")),
            walk_type=int(wt) if wt.isdigit() else 0,
        ))
    return WalkingRoute(
        distance_m=int(path.get("distance", 0) or 0),
        duration_s=int((path.get("cost", {}) or {}).get("duration", 0) or 0),
        steps=steps,
    )


def gps_to_gcj02(locations: list[tuple[float, float]],
                 key: str | None = None) -> list[tuple[float, float]]:
    """v3 坐标转换: GPS(WGS84) → 高德(GCJ02). 最多 40 对."""
    locs = "|".join(f"{lon:.6f},{lat:.6f}" for lon, lat in locations)
    data = _get(CONVERT_V3, {"locations": locs, "coordsys": "gps"}, key=key)
    out: list[tuple[float, float]] = []
    for pair in data.get("locations", "").split(";"):
        if "," not in pair:
            continue
        lon, lat = pair.split(",", 1)
        try:
            out.append((float(lon), float(lat)))
        except ValueError:
            continue
    return out

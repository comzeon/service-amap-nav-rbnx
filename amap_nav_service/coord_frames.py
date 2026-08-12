# SPDX-License-Identifier: MulanPSL-2.0
"""coord_frames — WGS84 <-> GCJ02 <-> UTM.

高德全链路是 GCJ02（火星坐标, 偏移可达数百米）；GPS/RTK 输出 WGS84。
严禁混用两种坐标系。UTM 投影懒加载 pyproj。
"""
from __future__ import annotations

import math

_A = 6378245.0
_EE = 0.00669342162296594323


def _out_of_china(lon: float, lat: float) -> bool:
    return not (72.004 <= lon <= 137.8347 and 0.8293 <= lat <= 55.8271)


def _transform_lat(x: float, y: float) -> float:
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
    return ret


def _transform_lon(x: float, y: float) -> float:
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
    return ret


def wgs84_to_gcj02(lon: float, lat: float) -> tuple[float, float]:
    """WGS84 -> GCJ02 (高德坐标系)."""
    if _out_of_china(lon, lat):
        return lon, lat
    dlat = _transform_lat(lon - 105.0, lat - 35.0)
    dlng = _transform_lon(lon - 105.0, lat - 35.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - _EE * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((_A * (1 - _EE)) / (magic * sqrtmagic) * math.pi)
    dlng = (dlng * 180.0) / (_A / sqrtmagic * math.cos(radlat) * math.pi)
    return lon + dlng, lat + dlat


def gcj02_to_wgs84(lon: float, lat: float, iters: int = 3) -> tuple[float, float]:
    """GCJ02 -> WGS84 迭代反算（高德不提供反向 API）. 3 次迭代亚米级误差."""
    if _out_of_china(lon, lat):
        return lon, lat
    iters = max(1, iters)
    glon, glat = lon, lat
    for _ in range(iters):
        glon2, glat2 = wgs84_to_gcj02(glon, glat)
        glon -= glon2 - lon
        glat -= glat2 - lat
    return glon, glat


def to_utm(lon: float, lat: float, zone: int | None = None) -> tuple[float, float, int]:
    """WGS84 lon/lat -> UTM (x, y, zone) 米. 懒加载 pyproj.

    仅支持北半球 (EPSG:326xx); 中国全境适用. zone 自动推导并 clamp 到 1-60.
    """
    from pyproj import Transformer  # noqa: PLC0415
    if zone is None:
        zone = min(int((lon + 180.0) // 6) + 1, 60)
    zone = max(1, min(zone, 60))
    t = Transformer.from_crs("EPSG:4326", f"EPSG:326{zone:02d}", always_xy=True)
    x, y = t.transform(lon, lat)
    return x, y, zone


def from_utm(x: float, y: float, zone: int) -> tuple[float, float]:
    """UTM (x, y, zone) -> WGS84 lon/lat."""
    from pyproj import Transformer  # noqa: PLC0415
    t = Transformer.from_crs(f"EPSG:326{zone:02d}", "EPSG:4326", always_xy=True)
    return t.transform(x, y)


def haversine_m(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    """球面距离（米）, 用于校验/兜底."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))

# service-amap-nav-rbnx — 高德辅助的开放场景 A→B 导航服务

`service-amap-nav-rbnx` 是 robonix 生态的**独立服务仓**（对齐 `service-navigation-rbnx` 模式）：
为机器狗补齐开放场景（城市街区/人行道）下 A→B 的**全局语义路径**缺口。

它调用高德 v5 步行路线规划拿到公里级"走哪条路"，转 UTM 执行帧后**逐段驱动现有
`navigate` 能力**（provider 可换：Webots sim = simple_nav，真机 = nav2），过街段由
人工闸门确认（Phase 2 升级视觉红绿灯检测）。**局部避障（厘米级"怎么走不撞"）永远由
下层 navigate 的 A* + costmap 负责，本服务不做**。

> 设计文档: 见 `2026-08-12-amap-nav-design.md`（Obsidian: Hermes/Robonix-Projects/）。
> 实施计划: `2026-08-12-amap-nav-plan.md`。契约随仓自带，**框架仓 comzeon/robonix 零改动**。

## 架构

```
┌─ 语义层（Phase 2, 可选）───────────────────────────────┐
│ 用户(语音/文字) → robonix pilot (LLM) → 官方高德MCP      │
│        → 终点语义解析(名称/地址→坐标) → 任务下发           │
└──────────────────────┬────────────────────────────────┘
                       │ 终点坐标 + 任务ID
┌──────────────────────▼────────────────────────────────┐
│ amap_nav 服务（本仓）                                    │
│  ├─ amap_client   ：高德 Web API 封装（纯 HTTP）        │
│  ├─ route_planner ：步行路线 → walk_type 严格白名单 →     │
│  │                 过街段标记 → 抽稀(长段插值, ≤40m)      │
│  ├─ coord_frames  ：GCJ02↔WGS84↔UTM 全局执行帧          │
│  ├─ executor      ：逐段 PoseStamped 下发 + 过街闸门 +   │
│  │                 偏离检测(默认20m) + 取消短路           │
│  └─ state         ：任务状态机                          │
└──────────────────────┬────────────────────────────────┘
                       │ PoseStamped goal（UTM 全局帧）
┌──────────────────────▼────────────────────────────────┐
│ 下层 navigate 能力: A* + costmap + follower             │
│ (sim=simple_nav / 真机=nav2, ATLAS 自动发现)             │
└───────────────────────────────────────────────────────┘
```

## 能力表面（MCP 工具）

| 契约 | 行为 |
| --- | --- |
| `robonix/service/navigation/amap_goal` | 开始 A→B 任务：终点 GCJ02 坐标（地址解析 Phase 2）→ 高德规划 → 白名单过滤 → UTM → 逐段执行 |
| `robonix/service/navigation/amap_status` | 任务状态（空 task_id = 最近任务）：known/state/detail/current_step |
| `robonix/service/navigation/amap_cancel` | 取消任务（空 = 最近）：**立即停止下发新 goal**（安全关键） |
| `robonix/service/navigation/amap_crossing_confirm` | 过街闸门人工确认（proceed=true/false；Phase 1 人工，Phase 2 视觉自动） |

IDL 与契约在本仓 `capabilities/lib/navigation/amap/srv/*.srv` + `capabilities/navigation/*.v1.toml`。

## 一次任务的数据流

1. `amap_goal(dest_gcj02, confirm_medium_energy)`：起点取 `AMAP_ORIGIN_WGS84`（GPS/RTK）→ 转 GCJ02
2. 距离护栏（默认 20km）→ 高德 `v5/direction/walking?show_fields=cost,navi,polyline`
3. `walk_type` **严格白名单**（不在 `{0,1,3,4,6,7,20,21,22,23}` 一律拒绝，宁停勿闯——天桥/台阶可走，扶梯/索道/轮渡拒绝）
4. 含过街/天桥/台阶段且未确认 → 拒绝任务（`confirm_medium_energy=true` 需人工批准）
5. 航点 GCJ02→WGS84→UTM（固定带号）→ `executor` 每 tick 下发一段
6. 过街段 → 停车 → `amap_crossing_confirm` 闸门（90s 超时自动 FAILED）
7. 偏离路线 > 20m → REPLANNING；发完 → ARRIVED → DONE；go() 异常/确认拒绝 → FAILED

## 配置

环境变量（部署时注入，**key 不落盘明文**）：

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `AMAP_WEB_KEY` | — | 高德 Web服务 API key（必填；`restapi.amap.com` 与官方 MCP 共用） |
| `AMAP_ORIGIN_WGS84` | — | Phase 1 起点 `lon,lat`（真机 P1.7 改为 RTK 话题订阅） |
| `AMAP_POSE_FILE` | — | 可选位姿文件 `x,y`（UTM），ticker 每 tick 读，供偏离检测 |
| `AMAP_MAX_ROUTE_KM` | 20.0 | 任务距离上限（护栏） |

`config/amap_nav_params.yaml`（部署仓 manifest 可覆盖）：`cross_track_threshold_m`（默认 20）、
`crossing_wait_timeout_s`（默认 90）、`max_route_km`（默认 20）。

## 坐标系统（关键坑）

高德全链路是 **GCJ02（火星坐标）**，GPS/RTK 是 WGS84，执行帧是 UTM——三者严禁混用。
转换全部本地完成：WGS84↔GCJ02 公开算法（往返 <1m），UTM 用 pyproj（固定带号）。
高德 `v3/assistant/coordinate/convert` 仅作兜底校验（一次最多 40 对）。

## 测试

```sh
# 本机（Windows uv.exe via WSL interop; 无 uv 时用任意 python3.10+）
/mnt/d/uv-x86_64-pc-windows-msvc/uv.exe run --no-project --with pyproj \
    python -m unittest discover -s tests
# 部署环境（构建后, 有 uv 时）
uv sync && uv run --with pyproj python -m unittest discover -s tests
```

当前 52 个用例：坐标往返、API 解析、白名单/抽稀/插值、状态机、过街闸门/偏离/取消、
service 层失败分支（mock codegen 依赖）。

真实 API 冒烟（需 `AMAP_WEB_KEY`）：

```sh
export AMAP_WEB_KEY='<你的key>' WSLENV=AMAP_WEB_KEY   # WSL→Windows 进程传环境变量
uv run --no-project --with pyproj python - <<'PY'
from amap_nav_service.amap_client import walking_route
from amap_nav_service import route_planner as rp, coord_frames as cf
route = walking_route((116.397428, 39.90923), (116.427104, 39.905152))
wps = rp.plan(route)
print(route.distance_m, len(wps), sorted({w.seg_type for w in wps}))
PY
```

## 状态

- **P1 ✅**：服务实现 + 契约 + 单测（52/52）+ 真实 API 冒烟（天安门→北京站 4.4km → 166 航点）。
- **P1.7 ⏳**：真机（Lite3）：RTK 接入（替换 `_current_origin_wgs84`）、部署仓 manifest url 引用、园区分级试跑。
- **P2 🔜**：pilot MCP 语义层（官方高德 MCP 15 工具已实测可用）+ 红绿灯视觉检测（过街闸门自动）。
- **P3 🔜**：无障碍盲道图层（`seg_type=tactile` 已预留，OSM/自建数据源可插拔）。

## 许可证

MulanPSL-2.0

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用户自定义航线：把用户在二维道路预览上点击的点，规划成一条沿道路连通的 AirSim 航线。

设计目标（与现有固定航线机制完全兼容）：
1. 复用 CARLA 的道路拓扑 get_topology() 构建无向加权图；
2. 点击点先由 CARLA 投影到最近的 Driving 车道航点（精确的"路上位置"），
   再在道路图上找最近的**边**并把锚点插进去（v2：精确吸附到边，而不是只落路口节点）；
3. 用 Dijkstra 在道路图上求相邻锚点之间的最短路径；
4. 拼接成 CARLA 坐标折线后，复用 road_cruise 的转弯平滑 / 路径加密 / Z 地形跟随，
   生成一条「只调用一次 moveOnPathAsync」的完整航线，并存成自定义航线 JSON。

图算法部分不依赖 carla / airsim，可在无仿真环境下单独单元测试。
"""

from __future__ import annotations

import heapq
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from road_cruise import (
    CruiseRoute,
    CruiseWaypoint,
    _densify_by_spacing,
    _smooth_route_corners,
    map_profile,
    normalize_map_name,
)

CUSTOM_ROUTE_DIR = Path(__file__).resolve().parent / "recordings" / "drone_routes" / "custom"
# 点击点距最近道路边的投影距离超过该值时给出告警，避免用户点到人行道 / 建筑附近。
SNAP_TOLERANCE_METERS = 50.0
# 锚点与图端点近于此距离时直接复用端点，避免拆出一堆零长度的碎边。
ANCHOR_MERGE_TOLERANCE_METERS = 0.75
# 不同道路上的边额外加一点"惩罚距离"：同一条路上的边最多可以远 25m 仍然胜出，
# 这样路口处（图节点按坐标合并后 road_id 只保留首次出现的值）也不会吸到垂直的另一条路。
OFF_ROAD_PENALTY_METERS = 25.0
# 同一条路但不同车道：轻微惩罚，优先本车道。
OFF_LANE_PENALTY_METERS = 1.0
# 转 AirSim 航线时相邻航点的最大间距，防止 AirSim 抄近路穿楼。
DENSIFY_MAX_SPACING = 4.0


# ---------------------------------------------------------------------------
# 图构建与最短路径
# ---------------------------------------------------------------------------

def _waypoint_node_key(wp: Any) -> Tuple[Any, ...]:
    """图节点的去重键：仅按平面坐标合并，保证交叉路口处的拓扑边连通。

    注意：不能像 road_cruise._waypoint_key 那样带 road_id/lane_id，
    否则同一路口不同 road_id 的边端点会被拆成互不连通的节点。
    """
    loc = wp.transform.location
    return (
        round(float(loc.x), 1),
        round(float(loc.y), 1),
    )


def build_topology_graph(carla_map: Any) -> Tuple[
    Dict[int, List[Tuple[int, float]]],
    Dict[int, Tuple[float, float, float]],
    Dict[int, Dict[str, int]],
]:
    """由 CARLA get_topology() 构建无向加权图。

    返回三元组 (adjacency, nodes, meta)：
    - adjacency: node_id -> [(neighbor_id, weight), ...]
    - nodes:     node_id -> (x, y, z)  CARLA 坐标
    - meta:      node_id -> {"road_id", "section_id", "lane_id"}
    """
    try:
        topology = carla_map.get_topology()
    except Exception:
        topology = []

    node_ids: Dict[Tuple[Any, ...], int] = {}
    nodes: Dict[int, Tuple[float, float, float]] = {}
    meta: Dict[int, Dict[str, int]] = {}
    adjacency: Dict[int, List[Tuple[int, float]]] = {}

    def intern(wp: Any) -> int:
        key = _waypoint_node_key(wp)
        existing = node_ids.get(key)
        if existing is not None:
            return existing
        idx = len(nodes)
        loc = wp.transform.location
        nodes[idx] = (float(loc.x), float(loc.y), float(loc.z))
        meta[idx] = {
            "road_id": int(getattr(wp, "road_id", -1)),
            "section_id": int(getattr(wp, "section_id", -1)),
            "lane_id": int(getattr(wp, "lane_id", 0)),
        }
        node_ids[key] = idx
        adjacency[idx] = []
        return idx

    for start_wp, end_wp in topology:
        a = intern(start_wp)
        b = intern(end_wp)
        if a == b:
            continue
        weight = max(1e-6, math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1]))
        if all(neighbor != b for neighbor, _ in adjacency[a]):
            adjacency[a].append((b, weight))
        if all(neighbor != a for neighbor, _ in adjacency[b]):
            adjacency[b].append((a, weight))

    return adjacency, nodes, meta


def nearest_graph_node(
    nodes: Dict[int, Tuple[float, float, float]],
    meta: Dict[int, Dict[str, int]],
    x: float,
    y: float,
    preferred_road_id: Optional[int] = None,
    preferred_lane_id: Optional[int] = None,
) -> Tuple[Optional[int], float]:
    """找最近的图节点，优先落在同一车道 / 同一道路上，避免误吸到相邻道路。"""
    if not nodes:
        return None, float("inf")

    def distance(node_id: int) -> float:
        nx, ny, _ = nodes[node_id]
        return math.hypot(nx - x, ny - y)

    candidates = list(nodes.keys())
    if preferred_road_id is not None:
        same_road = [i for i in candidates if meta[i]["road_id"] == preferred_road_id]
        if same_road:
            if preferred_lane_id is not None:
                same_lane = [i for i in same_road if meta[i]["lane_id"] == preferred_lane_id]
                if same_lane:
                    candidates = same_lane
                else:
                    candidates = same_road
            else:
                candidates = same_road

    best = min(candidates, key=distance)
    return best, distance(best)


def project_point_to_segment(
    px: float, py: float,
    ax: float, ay: float,
    bx: float, by: float,
) -> Tuple[float, float, float, float]:
    """把点 P 投影到线段 AB 上。

    返回 (t, qx, qy, distance)：
    - t:      投影参数，已夹到 [0, 1]（超出端点时贴到端点上）
    - (qx,qy): 投影点坐标
    - distance: P 到投影点的平面距离
    退化线段（A 与 B 重合）按端点到点距离处理，不会除零。
    """
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return 0.0, ax, ay, math.hypot(px - ax, py - ay)

    t = ((px - ax) * dx + (py - ay) * dy) / length_sq
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0

    qx = ax + t * dx
    qy = ay + t * dy
    return t, qx, qy, math.hypot(px - qx, py - qy)


def iter_undirected_edges(adjacency: Dict[int, List[Tuple[int, float]]]):
    """去重遍历无向边（每条边只产出 a < b 的一次）。"""
    for a in sorted(adjacency.keys()):
        for b, _weight in adjacency[a]:
            if a < b:
                yield a, b


def nearest_graph_edge(
    adjacency: Dict[int, List[Tuple[int, float]]],
    nodes: Dict[int, Tuple[float, float, float]],
    meta: Dict[int, Dict[str, int]],
    x: float,
    y: float,
    preferred_road_id: Optional[int] = None,
    preferred_lane_id: Optional[int] = None,
) -> Optional[Tuple[int, int, float, float, float, float]]:
    """在道路图上找最近的**边**（按投影距离），返回 (a, b, t, qx, qy, distance)。

    与"最近节点"相比，投影到边上可以把锚点放在路段中间，而不是只能落在路口节点上。
    带 road / lane 偏好：优先同一条道路（其次同一条车道）上的边，避免吸附到平行道路。
    图上没有任何边时返回 None。
    """
    best: Optional[Tuple[int, int, float, float, float, float]] = None
    best_score = float("inf")

    for a, b in iter_undirected_edges(adjacency):
        ax, ay, _az = nodes[a]
        bx, by, _bz = nodes[b]
        t, qx, qy, distance = project_point_to_segment(x, y, ax, ay, bx, by)

        penalty = 0.0
        if preferred_road_id is not None:
            same_road = (
                meta[a]["road_id"] == preferred_road_id
                or meta[b]["road_id"] == preferred_road_id
            )
            if same_road:
                same_lane = (
                    preferred_lane_id is None
                    or meta[a]["lane_id"] == preferred_lane_id
                    or meta[b]["lane_id"] == preferred_lane_id
                )
                if not same_lane:
                    penalty = OFF_LANE_PENALTY_METERS
            else:
                penalty = OFF_ROAD_PENALTY_METERS

        score = distance + penalty
        if score < best_score:
            best_score = score
            best = (a, b, t, qx, qy, distance)

    return best


def insert_anchor_node(
    adjacency: Dict[int, List[Tuple[int, float]]],
    nodes: Dict[int, Tuple[float, float, float]],
    meta: Dict[int, Dict[str, int]],
    a: int,
    b: int,
    x: float,
    y: float,
    z: float,
    road_id: Optional[int] = None,
    section_id: Optional[int] = None,
    lane_id: Optional[int] = None,
    merge_tolerance: float = ANCHOR_MERGE_TOLERANCE_METERS,
) -> int:
    """把锚点插入到边 (a, b) 上：拆分该边，返回锚点节点 id。

    - 锚点离某个端点足够近时直接复用该端点，不制造零长度碎边；
    - 拆分后 (a, b) 被 (a, anchor) 与 (anchor, b) 取代，权重按实际长度重算，
      因此总路径长度不受影响。
    """
    for endpoint in (a, b):
        ex, ey, _ez = nodes[endpoint]
        if math.hypot(ex - float(x), ey - float(y)) <= merge_tolerance:
            return endpoint

    new_id = (max(nodes) + 1) if nodes else 0
    template = meta.get(a, {})
    nodes[new_id] = (float(x), float(y), float(z))
    meta[new_id] = {
        "road_id": int(road_id if road_id is not None else template.get("road_id", -1)),
        "section_id": int(section_id if section_id is not None else template.get("section_id", -1)),
        "lane_id": int(lane_id if lane_id is not None else template.get("lane_id", 0)),
    }
    adjacency[new_id] = []

    # 拆掉原来的 (a, b) 边，换成 a-anchor 与 anchor-b。
    adjacency[a] = [(n, w) for (n, w) in adjacency[a] if n != b]
    adjacency[b] = [(n, w) for (n, w) in adjacency[b] if n != a]
    for endpoint in (a, b):
        weight = math.hypot(nodes[endpoint][0] - float(x), nodes[endpoint][1] - float(y))
        weight = max(1e-6, weight)
        adjacency[new_id].append((endpoint, weight))
        adjacency[endpoint].append((new_id, weight))

    return new_id


def shortest_path(
    adjacency: Dict[int, List[Tuple[int, float]]],
    start: int,
    end: int,
) -> Optional[List[int]]:
    """Dijkstra 求两节点间最短路径，返回节点 id 序列；不连通返回 None。"""
    if start == end:
        return [start]

    distances = {start: 0.0}
    previous: Dict[int, int] = {}
    visited = set()
    heap = [(0.0, start)]

    while heap:
        dist, current = heapq.heappop(heap)
        if current in visited:
            continue
        visited.add(current)
        if current == end:
            break
        for neighbor, weight in adjacency.get(current, []):
            candidate = dist + weight
            if candidate < distances.get(neighbor, float("inf")):
                distances[neighbor] = candidate
                previous[neighbor] = current
                heapq.heappush(heap, (candidate, neighbor))

    if end not in distances:
        return None
    path = [end]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# 吸附与规划
# ---------------------------------------------------------------------------

def snap_to_road_waypoint(carla_map: Any, x: float, y: float, z: float = 0.0) -> Optional[Any]:
    """把点击点投影到最近的 Driving 车道航点；延迟导入 carla 以便图算法单独测试。"""
    import carla

    location = carla.Location(x=float(x), y=float(y), z=float(z))
    waypoint = None
    try:
        waypoint = carla_map.get_waypoint(
            location, project_to_road=True, lane_type=carla.LaneType.Driving
        )
    except (TypeError, AttributeError):
        waypoint = carla_map.get_waypoint(location, project_to_road=True)
    if waypoint is None:
        try:
            waypoint = carla_map.get_waypoint(location, project_to_road=True)
        except Exception:
            waypoint = None
    return waypoint


def plan_route_carla(
    carla_map: Any,
    points: Sequence[Sequence[float]],
) -> Tuple[Optional[List[Tuple[float, float, float]]], str, List[str]]:
    """把依次点击的 CARLA XY 点规划成沿道路连通的 CARLA 坐标折线。

    吸附策略 v2（精确到边）：
    1. 先用 CARLA 把点击点投影到最近的车道中心线 —— 得到精确的"路上位置"；
    2. 再在道路图上找最近的**边**（不是最近的节点），把锚点插进这条边；
    因此航线起终点落在用户点的位置附近，而不是过去那个"最近拓扑节点"（路口，误差可达数米）。

    返回 (polyline, error, warnings)：
    - polyline: [(x, y, z), ...]，成功时为非空列表；
    - error:    失败原因（成功时为空字符串）；
    - warnings: 吸附距离偏大等提示信息。
    """
    if len(points) < 2:
        return None, "自定义航线至少需要起点和终点两个点", []

    adjacency, nodes, meta = build_topology_graph(carla_map)
    if not nodes:
        return None, "当前地图没有可用的道路拓扑", []

    warnings: List[str] = []
    anchor_node_ids: List[int] = []
    for index, point in enumerate(points):
        x = float(point[0])
        y = float(point[1])
        waypoint = snap_to_road_waypoint(carla_map, x, y)
        if waypoint is None:
            return None, f"第 {index + 1} 个点无法吸附到道路，请点在道路附近", warnings
        loc = waypoint.transform.location
        snap_x = float(loc.x)
        snap_y = float(loc.y)
        snap_z = float(loc.z)
        road_id = int(getattr(waypoint, "road_id", -1))
        section_id = int(getattr(waypoint, "section_id", -1))
        lane_id = int(getattr(waypoint, "lane_id", 0))

        edge = nearest_graph_edge(
            adjacency, nodes, meta, snap_x, snap_y,
            preferred_road_id=road_id, preferred_lane_id=lane_id,
        )
        if edge is None:
            return None, f"第 {index + 1} 个点附近没有可用的道路边", warnings
        edge_a, edge_b, _t, _qx, _qy, edge_distance = edge
        if edge_distance > SNAP_TOLERANCE_METERS:
            warnings.append(
                f"第 {index + 1} 个点距最近道路边 {edge_distance:.1f}m，可能偏离道路网络"
            )
        # 锚点直接用 CARLA 投影后的路面位置（精确到车道中心线），
        # 通过拆分最近的边接入图，保证 Dijkstra 能找到连通路径。
        anchor_node_ids.append(
            insert_anchor_node(
                adjacency, nodes, meta, edge_a, edge_b,
                snap_x, snap_y, snap_z,
                road_id=road_id, section_id=section_id, lane_id=lane_id,
            )
        )

    full_path: List[int] = [anchor_node_ids[0]]
    for start, end in zip(anchor_node_ids, anchor_node_ids[1:]):
        segment = shortest_path(adjacency, start, end)
        if segment is None:
            return None, "所选点之间的道路不连通，请调整途经点", warnings
        if segment and segment[0] == full_path[-1]:
            full_path.extend(segment[1:])
        else:
            full_path.extend(segment)

    polyline = [(nodes[i][0], nodes[i][1], nodes[i][2]) for i in full_path]
    return polyline, "", warnings


# ---------------------------------------------------------------------------
# 转 AirSim 航线 + 保存 / 读取
# ---------------------------------------------------------------------------

def _add_headings(
    polyline: Sequence[Tuple[float, float, float]],
) -> List[Tuple[float, float, float, float]]:
    """给折线补上每点的 yaw（度）。执行时 ForwardOnly 按路径切线飞行，yaw 仅用于记录。"""
    result: List[Tuple[float, float, float, float]] = []
    for index, (x, y, z) in enumerate(polyline):
        if index + 1 < len(polyline):
            nx, ny = polyline[index + 1][0], polyline[index + 1][1]
        elif index > 0:
            nx, ny = polyline[index - 1][0], polyline[index - 1][1]
        else:
            nx, ny = x + 1.0, y
        yaw = math.degrees(math.atan2(ny - y, nx - x))
        result.append((float(x), float(y), float(z), yaw))
    return result


def carla_polyline_to_airsim_route(
    map_name: str,
    polyline: Sequence[Tuple[float, float, float]],
    altitude: float,
    airsim_xy_offset: Tuple[float, float],
    reference_ground_z: Optional[float] = None,
) -> CruiseRoute:
    """把 CARLA 折线转成可执行的 AirSim 航线，复用固定航线的平滑 / 加密 / Z 地形跟随逻辑。"""
    if len(polyline) < 2:
        raise ValueError("航线至少需要两个点")

    headed = _add_headings(polyline)
    smoothed = _smooth_route_corners(headed)
    densified = _densify_by_spacing(smoothed, max_spacing=DENSIFY_MAX_SPACING)

    ox, oy = airsim_xy_offset
    ref_z = float(reference_ground_z) if reference_ground_z is not None else 0.0
    waypoints = [
        CruiseWaypoint(
            x=p[0] + ox,
            y=p[1] + oy,
            # CARLA Z 向上、AirSim NED Z 向下；跟随道路高程保持相对离地高度。
            z=-abs(float(altitude)) - (float(p[2]) - ref_z),
            yaw=p[3],
        )
        for p in densified
    ]
    return CruiseRoute(
        map_name=normalize_map_name(map_name),
        source="custom_user_route",
        waypoints=waypoints,
        profile=map_profile(normalize_map_name(map_name)),
    )


def save_custom_route(route: CruiseRoute, route_id: str) -> Path:
    """把自定义航线保存为固定航线同构的 JSON 资产（放在 custom 子目录，不影响固定航线）。"""
    CUSTOM_ROUTE_DIR.mkdir(parents=True, exist_ok=True)
    path = CUSTOM_ROUTE_DIR / f"{route.map_name}_custom_{route_id}.json"
    data = {
        "map": route.map_name,
        "source": route.source,
        "route_id": route_id,
        "altitude": round(abs(route.waypoints[0].z), 3) if route.waypoints else None,
        "waypoints": [
            {"x": round(wp.x, 4), "y": round(wp.y, 4), "z": round(wp.z, 4), "yaw": wp.yaw}
            for wp in route.waypoints
        ],
    }
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def prepend_start_connector(
    route: CruiseRoute,
    start_x: float,
    start_y: float,
    spacing: float = 4.0,
) -> CruiseRoute:
    """在航线首航点前补一段从当前无人机位置出发的连接段。

    自定义航线的起点由用户点击决定，可能不在无人机出生点；
    若让 AirSim 直接从出生点飞向远处首航点会直线穿楼，因此补上密化的连接段。
    """
    first = route.waypoints[0]
    distance = math.hypot(first.x - start_x, first.y - start_y)
    if distance < 0.05:
        return route
    segments = max(1, math.ceil(distance / spacing))
    yaw = math.degrees(math.atan2(first.y - start_y, first.x - start_x))
    connector = [
        CruiseWaypoint(
            x=start_x + (first.x - start_x) * index / segments,
            y=start_y + (first.y - start_y) * index / segments,
            z=first.z,
            yaw=yaw,
        )
        for index in range(segments)
    ]
    return CruiseRoute(
        map_name=route.map_name,
        source=f"start_connector+{route.source}",
        waypoints=[*connector, *route.waypoints],
        profile=route.profile,
    )


def load_custom_cruise_route(map_name: str, route_id: str) -> CruiseRoute:
    """读取已保存的自定义航线，结构上与固定航线一致，可直接交给 RoadCruiseController 执行。"""
    short_name = normalize_map_name(map_name)
    path = CUSTOM_ROUTE_DIR / f"{short_name}_custom_{route_id}.json"
    if not path.is_file():
        raise RuntimeError(f"未找到自定义航线: {path.name}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        waypoints = [
            CruiseWaypoint(
                x=float(point["x"]),
                y=float(point["y"]),
                z=float(point["z"]),
                yaw=None if point.get("yaw") is None else float(point["yaw"]),
            )
            for point in data["waypoints"]
        ]
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"自定义航线文件格式错误: {path.name}") from exc

    if len(waypoints) < 2:
        raise RuntimeError(f"自定义航线至少需要两个航点: {path.name}")
    return CruiseRoute(
        map_name=short_name,
        source=f"custom_file:{path.name}",
        waypoints=waypoints,
        profile=map_profile(short_name),
    )


def list_custom_routes(map_name: str) -> List[str]:
    """列出某张地图已保存的自定义航线 id。"""
    short_name = normalize_map_name(map_name)
    prefix = f"{short_name}_custom_"
    if not CUSTOM_ROUTE_DIR.is_dir():
        return []
    result = []
    for path in CUSTOM_ROUTE_DIR.glob(f"{prefix}*.json"):
        stem = path.stem
        route_id = stem[len(prefix):]
        result.append(route_id)
    return sorted(result)

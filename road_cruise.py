#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Road-following drone cruise routes for CarlaAir dataset collection.

The module builds a route from the current CARLA map instead of hard-coding
Town coordinates. That keeps Town01, Town02, Town03, Town04, Town05,
Town10HD, and future maps covered by the same road-following logic.
"""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, List, Optional, Sequence, Tuple


DEFAULT_MAPS = ("Town01", "Town02", "Town03", "Town04", "Town05", "Town10HD")
FIXED_ROUTE_DIR = Path(__file__).resolve().parent / "recordings" / "drone_routes"


@dataclass(frozen=True)
class CruiseWaypoint:
    """AirSim NED waypoint."""

    x: float
    y: float
    z: float
    yaw: Optional[float] = None


@dataclass(frozen=True)
class RouteProfile:
    """Per-map route generation parameters."""

    sampling_resolution: float
    max_waypoints: int
    min_spacing: float
    altitude_bias: float = 0.0


@dataclass
class CruiseRoute:
    """Generated route metadata and waypoints."""

    map_name: str
    source: str
    waypoints: List[CruiseWaypoint]
    profile: RouteProfile


class RoadCruiseController:
    """将一次性生成的完整航线提交为一个 AirSim 飞行任务。"""

    def __init__(
        self,
        airsim_client: Any,
        route: CruiseRoute,
        speed: float,
    ) -> None:
        self.client = airsim_client
        self.route = route
        self.speed = float(speed)
        self.index = 0
        self.task: Optional[Any] = None

    def start(self) -> Any:
        """一次提交全程固定航点，避免巡航中不断覆盖当前飞行任务。"""
        if len(self.route.waypoints) < 2:
            raise RuntimeError('巡航路线至少需要两个航点')

        import airsim as _airsim

        path = [
            _airsim.Vector3r(waypoint.x, waypoint.y, waypoint.z)
            for waypoint in self.route.waypoints
        ]
        # ForwardOnly 让 AirSim 以相邻路径点连线为前进方向。
        # 路线在生成阶段已补足拐角和直线插值点，因此机头会沿离散切线平滑转向。
        self.task = self.client.moveOnPathAsync(
            path,
            self.speed,
            drivetrain=_airsim.DrivetrainType.ForwardOnly,
            # ForwardOnly 不能与默认的“偏航速率”模式并用；否则路径任务会被拒绝或停在原地。
            # 非速率模式下的 0 度偏移表示沿路径切线朝前，而不是额外偏向某个方向。
            yaw_mode=_airsim.YawMode(False, 0.0),
            # 由 AirSim 自动选择前视点，避免固定前视距离在不同巡航速度下产生过冲。
            lookahead=-1,
            adaptive_lookahead=0,
        )
        # Future 仅用于保留任务引用。巡航期间不能调用 join()，
        # 因为 msgpackrpc 会占用同一个客户端事件循环，妨碍取消和降落指令发送。
        return self.task


def nearest_route_index(waypoints: Sequence[CruiseWaypoint], x: float, y: float) -> Optional[int]:
    """返回距指定平面坐标最近的路线航点索引。"""
    if not waypoints:
        return None
    return min(
        range(len(waypoints)),
        key=lambda index: math.hypot(waypoints[index].x - x, waypoints[index].y - y),
    )


def nearest_route_indices(
    waypoints: Sequence[CruiseWaypoint], x: float, y: float, tolerance: float = 0.5,
) -> List[int]:
    """返回所有同样接近目标点的航点，保留去程和返程的重合位置。"""
    index = nearest_route_index(waypoints, x, y)
    if index is None:
        return []
    minimum_distance = math.hypot(waypoints[index].x - x, waypoints[index].y - y)
    return [
        candidate
        for candidate, waypoint in enumerate(waypoints)
        if math.hypot(waypoint.x - x, waypoint.y - y) <= minimum_distance + tolerance
    ]


def route_segment_between(
    waypoints: Sequence[CruiseWaypoint], start_index: int, end_index: int,
) -> List[CruiseWaypoint]:
    """返回两航点间的原路线片段，允许沿来路反向返回。"""
    if not waypoints:
        return []
    if not 0 <= start_index < len(waypoints) or not 0 <= end_index < len(waypoints):
        raise IndexError('航点索引超出路线范围')
    if start_index <= end_index:
        return list(waypoints[start_index:end_index + 1])
    return list(reversed(waypoints[end_index:start_index + 1]))


def route_distance_between(
    waypoints: Sequence[CruiseWaypoint], start_index: int, end_index: int,
) -> float:
    """计算沿既定路线在两个航点之间飞行的三维距离。"""
    segment = route_segment_between(waypoints, start_index, end_index)
    return sum(
        math.sqrt(
            (next_point.x - point.x) ** 2
            + (next_point.y - point.y) ** 2
            + (next_point.z - point.z) ** 2
        )
        for point, next_point in zip(segment, segment[1:])
    )


def normalize_map_name(name: str) -> str:
    """Return the short map name, e.g. /Game/Carla/Maps/Town03 -> Town03."""
    if not name:
        return ""
    return str(name).replace("\\", "/").split("/")[-1]


def load_fixed_cruise_route(
    map_name: str,
    route_dir: Optional[Path] = None,
) -> CruiseRoute:
    """读取当前地图已标定的固定 AirSim 航线，不在运行时重新规划。"""
    short_map_name = normalize_map_name(map_name)
    base_dir = Path(route_dir) if route_dir is not None else FIXED_ROUTE_DIR
    route_path = base_dir / f"{short_map_name}_road_cruise_route.json"
    if not route_path.is_file():
        raise RuntimeError(f"未找到地图 {short_map_name} 的固定巡航航线: {route_path.name}")

    try:
        data = json.loads(route_path.read_text(encoding="utf-8"))
        if normalize_map_name(data.get("map", "")) != short_map_name:
            raise ValueError("航线文件地图名称不匹配")
        profile_data = data.get("profile", {})
        profile = RouteProfile(
            sampling_resolution=float(profile_data.get("sampling_resolution", 0.0)),
            max_waypoints=int(profile_data.get("max_waypoints", 0)),
            min_spacing=float(profile_data.get("min_spacing", 0.0)),
            altitude_bias=float(profile_data.get("altitude_bias", 0.0)),
        )
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
        raise RuntimeError(f"固定巡航航线文件格式错误: {route_path.name}") from exc

    if len(waypoints) < 2:
        raise RuntimeError(f"固定巡航航线至少需要两个航点: {route_path.name}")
    return CruiseRoute(
        map_name=short_map_name,
        source=f"fixed_file:{route_path.name}",
        waypoints=waypoints,
        profile=profile,
    )


def available_maps_from_client(client: Any) -> List[str]:
    """Read available map names from a CARLA client."""
    try:
        maps = [normalize_map_name(m) for m in client.get_available_maps()]
        maps = [m for m in maps if m and not m.endswith("_Opt")]
        return sorted(set(maps))
    except Exception:
        return []


def available_maps_from_files(base_dir: Optional[Path] = None) -> List[str]:
    """Find packaged Town maps from the CarlaAir directory."""
    if base_dir is None:
        base_dir = Path(__file__).resolve().parents[2]
    maps_dir = base_dir / "WindowsNoEditor" / "CarlaUE4" / "Content" / "Carla" / "Maps"
    if not maps_dir.exists():
        return list(DEFAULT_MAPS)

    names = []
    for path in maps_dir.glob("Town*.umap"):
        stem = path.stem
        if stem.endswith("_Opt"):
            continue
        names.append(stem)
    return sorted(set(names)) or list(DEFAULT_MAPS)


def available_maps(client: Optional[Any] = None, base_dir: Optional[Path] = None) -> List[str]:
    """Return maps known either by CARLA RPC or local packaged files."""
    names = available_maps_from_client(client) if client is not None else []
    if not names:
        names = available_maps_from_files(base_dir)
    return names


def resolve_map_name(client: Any, requested: str) -> str:
    """Resolve a user supplied map selector to an available map name."""
    requested = (requested or "").strip()
    if not requested:
        raise ValueError("empty map name")

    maps = available_maps(client)
    if not maps:
        return requested

    exact = [m for m in maps if m.lower() == requested.lower()]
    if exact:
        return exact[0]

    partial = [m for m in maps if requested.lower() in m.lower()]
    if len(partial) == 1:
        return partial[0]

    if partial:
        raise ValueError(f"map name '{requested}' is ambiguous: {partial}")

    raise ValueError(f"map '{requested}' not found. Available maps: {maps}")


def maybe_prompt_choice(prompt: str, choices: Sequence[str], default: str) -> str:
    """Prompt only when running interactively; otherwise return default."""
    if not _can_prompt():
        return default

    print(prompt)
    for i, choice in enumerate(choices, 1):
        suffix = " [default]" if choice == default else ""
        print(f"  {i}. {choice}{suffix}")

    raw = input("> ").strip()
    if not raw:
        return default
    if raw.isdigit():
        idx = int(raw) - 1
        if 0 <= idx < len(choices):
            return choices[idx]
    for choice in choices:
        if raw.lower() == choice.lower():
            return choice
    print(f"Invalid choice '{raw}', using {default}")
    return default


def choose_map_interactively(client: Any, default: Optional[str] = None) -> str:
    maps = available_maps(client)
    if not maps:
        maps = list(DEFAULT_MAPS)
    if default is None or default not in maps:
        default = maps[0]
    return maybe_prompt_choice("Choose CARLA map:", maps, default)


def choose_flight_mode_interactively(default: str = "auto") -> str:
    return maybe_prompt_choice(
        "Choose drone flight mode:",
        ("auto", "manual", "hover"),
        default if default in {"auto", "manual", "hover"} else "auto",
    )


def load_requested_map(client: Any, requested: Optional[str], interactive: bool = False) -> Tuple[Any, str]:
    """Optionally switch the CARLA world and return (world, short_map_name)."""
    if interactive and not requested:
        requested = choose_map_interactively(client)

    if requested:
        map_name = resolve_map_name(client, requested)
        print(f"[CARLA] Loading map: {map_name}")
        world = client.load_world(map_name)
        return world, normalize_map_name(world.get_map().name)

    world = client.get_world()
    return world, normalize_map_name(world.get_map().name)


def map_profile(map_name: str) -> RouteProfile:
    """Select route density for each packaged map."""
    profiles = {
        "Town01": RouteProfile(sampling_resolution=10.0, max_waypoints=180, min_spacing=10.0),
        "Town02": RouteProfile(sampling_resolution=10.0, max_waypoints=180, min_spacing=10.0),
        "Town03": RouteProfile(sampling_resolution=12.0, max_waypoints=220, min_spacing=12.0),
        "Town04": RouteProfile(sampling_resolution=16.0, max_waypoints=240, min_spacing=16.0),
        "Town05": RouteProfile(sampling_resolution=12.0, max_waypoints=240, min_spacing=12.0),
        "Town10HD": RouteProfile(sampling_resolution=12.0, max_waypoints=240, min_spacing=12.0),
    }
    return profiles.get(normalize_map_name(map_name), RouteProfile(12.0, 220, 12.0))


def build_road_cruise_route(
    world: Any,
    altitude: float,
    airsim_xy_offset: Tuple[float, float] = (0.0, 0.0),
    preferred_start_xy: Optional[Tuple[float, float]] = None,
    preferred_start_waypoint: Optional[Any] = None,
    reference_ground_z: Optional[float] = None,
) -> CruiseRoute:
    """Build a route over road waypoints in the current map."""
    carla_map = world.get_map()
    map_name = normalize_map_name(carla_map.name)
    profile = map_profile(map_name)

    points = _continuous_road_route(
        carla_map,
        profile,
        preferred_start_xy,
        preferred_start_waypoint,
    )
    source = "waypoint_next_out_and_back"
    if not points:
        # 已知无人机起点时禁止回退到出生点路线，否则 AirSim 会直线飞向数百米外的首航点。
        if preferred_start_xy is not None:
            raise RuntimeError(
                f"无法从无人机附近道路生成巡航路线（起点 {preferred_start_xy[0]:.1f}, "
                f"{preferred_start_xy[1]:.1f}）"
            )
        points = _spawn_points_route(carla_map, profile, preferred_start_xy)
        source = "spawn_points_out_and_back"
    if not points:
        raise RuntimeError(
            f"Cannot build continuous road cruise route for map {map_name}. "
            "Use --flight-mode hover/manual or check whether this map exposes CARLA road waypoints."
        )

    sampled = _sample_by_spacing(points, profile.min_spacing)
    sampled = _limit_count(sampled, max(4, profile.max_waypoints // 2))
    sampled = _out_and_back(sampled)
    # 道路中心线通常以较稀疏的离散点表示。对正常拐角插入二次曲线点，
    # 使飞行目标和机头切线在转弯时逐步变化，而不是一次跳转 90 度。
    sampled = _smooth_route_corners(sampled)
    # AirSim 会在相邻目标间直线飞行，必须加密整条路径而不仅是转角。
    sampled = _densify_by_spacing(sampled, max_spacing=4.0)
    ox, oy = airsim_xy_offset
    waypoints = [
        CruiseWaypoint(
            x=p[0] + ox,
            y=p[1] + oy,
            # CARLA Z 轴向上、AirSim NED Z 轴向下。以起飞位置道路高程为基准，
            # 让巡航高度跟随山路、桥梁等地形变化而保持相对离地高度。
            z=-abs(float(altitude) + profile.altitude_bias)
            - ((float(p[2]) - reference_ground_z) if len(p) > 3 and reference_ground_z is not None else 0.0),
            yaw=p[3] if len(p) > 3 else (p[2] if len(p) > 2 else None),
        )
        for p in sampled
    ]

    return CruiseRoute(map_name=map_name, source=source, waypoints=waypoints, profile=profile)


def save_route_preview(route: CruiseRoute, output_dir: Path) -> Path:
    """Save generated route coordinates for repeatability/debugging."""
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{route.map_name}_road_cruise_route.json"
    data = {
        "map": route.map_name,
        "source": route.source,
        "profile": {
            "sampling_resolution": route.profile.sampling_resolution,
            "max_waypoints": route.profile.max_waypoints,
            "min_spacing": route.profile.min_spacing,
            "altitude_bias": route.profile.altitude_bias,
        },
        "waypoints": [{"x": wp.x, "y": wp.y, "z": wp.z, "yaw": wp.yaw} for wp in route.waypoints],
    }

    import json

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    return path


def _continuous_road_route(
    carla_map: Any,
    profile: RouteProfile,
    preferred_start_xy: Optional[Tuple[float, float]],
    preferred_start_waypoint: Optional[Any] = None,
) -> List[Tuple[float, float, float]]:
    candidates = _candidate_waypoints(carla_map)
    if not candidates:
        return []

    if preferred_start_waypoint is not None:
        # 直接使用无人机垂直投影到道路后的航点，避免只从稀疏拓扑端点开始。
        start_key = _waypoint_key(preferred_start_waypoint)
        candidates = [
            preferred_start_waypoint,
            *[candidate for candidate in candidates if _waypoint_key(candidate) != start_key],
        ]
        candidates[1:] = sorted(
            candidates[1:],
            key=lambda wp: _waypoint_xy_distance(wp, preferred_start_xy),
        )
    elif preferred_start_xy is not None:
        candidates.sort(key=lambda wp: _waypoint_xy_distance(wp, preferred_start_xy))

    best: List[Tuple[float, float]] = []
    best_score = -1.0
    # 起飞前的路线搜索在请求线程中执行；限制候选数避免用户误以为程序卡死。
    # 已有精确投影航点时优先尝试它；没有投影航点时再扩大附近候选范围。
    max_candidates = min(
        len(candidates),
        16 if preferred_start_waypoint is not None else (64 if preferred_start_xy is not None else 24),
    )

    for wp in candidates[:max_candidates]:
        points = _follow_waypoints(
            wp,
            step=profile.sampling_resolution,
            max_points=max(12, profile.max_waypoints // 2),
        )
        if len(points) < 4:
            continue

        length = _polyline_length(points)
        start_distance = 0.0
        if preferred_start_xy is not None:
            start_distance = _point_distance(points[0], preferred_start_xy)
            # 首个目标太远时，AirSim 会跨越建筑直飞过去；这条路线不能使用。
            if start_distance > max(30.0, profile.sampling_resolution * 3.0):
                continue
        score = length - start_distance * 4.0
        if score > best_score:
            best = points
            best_score = score

    return best


def _candidate_waypoints(carla_map: Any) -> List[Any]:
    candidates: List[Any] = []
    seen = set()

    for sp in _spawn_transforms(carla_map):
        wp = _get_waypoint(carla_map, sp.location)
        if wp is None:
            continue
        key = _waypoint_key(wp)
        if key not in seen:
            seen.add(key)
            candidates.append(wp)

    try:
        topology = carla_map.get_topology()
    except Exception:
        topology = []

    for start, end in topology:
        for wp in (start, end):
            key = _waypoint_key(wp)
            if key not in seen:
                seen.add(key)
                candidates.append(wp)

    return candidates


def _follow_waypoints(start_wp: Any, step: float, max_points: int) -> List[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    seen = set()
    current = start_wp
    guard_limit = max_points * 2

    for _ in range(guard_limit):
        key = _waypoint_key(current)
        if key in seen and len(points) > 8:
            break
        seen.add(key)

        loc = current.transform.location
        p = (float(loc.x), float(loc.y), float(loc.z), float(current.transform.rotation.yaw))
        if not points or _point_distance(points[-1], p) >= max(1.0, step * 0.5):
            points.append(p)
        if len(points) >= max_points:
            break

        try:
            nexts = current.next(step)
        except Exception:
            break
        if not nexts:
            break

        current = _choose_next_waypoint(current, nexts, seen)

    return points


def _choose_next_waypoint(current: Any, nexts: Sequence[Any], seen: set) -> Any:
    current_yaw = float(current.transform.rotation.yaw)

    def score(wp: Any) -> float:
        yaw = float(wp.transform.rotation.yaw)
        visited_penalty = 1000.0 if _waypoint_key(wp) in seen else 0.0
        road_change_penalty = 20.0 if getattr(wp, "road_id", None) != getattr(current, "road_id", None) else 0.0
        return visited_penalty + road_change_penalty + abs(_normalize_angle(yaw - current_yaw))

    return min(nexts, key=score)


def _spawn_points_route(
    carla_map: Any,
    profile: RouteProfile,
    preferred_start_xy: Optional[Tuple[float, float]],
) -> List[Tuple[float, float, float]]:
    points = _spawn_points(carla_map)
    if not points:
        return []
    if preferred_start_xy is not None:
        points.sort(key=lambda p: _point_distance(p, preferred_start_xy))
    points = _limit_count(points, max(4, profile.max_waypoints // 2))
    return _out_and_back(points)


def _spawn_transforms(carla_map: Any) -> List[Any]:
    try:
        return list(carla_map.get_spawn_points())
    except Exception:
        return []


def _get_waypoint(carla_map: Any, location: Any) -> Optional[Any]:
    try:
        return carla_map.get_waypoint(location, project_to_road=True)
    except TypeError:
        try:
            return carla_map.get_waypoint(location)
        except Exception:
            return None
    except Exception:
        return None


def _waypoint_key(wp: Any) -> Tuple[Any, ...]:
    loc = wp.transform.location
    return (
        getattr(wp, "road_id", None),
        getattr(wp, "section_id", None),
        getattr(wp, "lane_id", None),
        round(float(getattr(wp, "s", 0.0)), 1),
        round(float(loc.x), 1),
        round(float(loc.y), 1),
    )


def _waypoint_xy_distance(wp: Any, xy: Tuple[float, float]) -> float:
    loc = wp.transform.location
    return math.hypot(float(loc.x) - xy[0], float(loc.y) - xy[1])


def _normalize_angle(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _polyline_length(points: Sequence[Sequence[float]]) -> float:
    if len(points) < 2:
        return 0.0
    return sum(_point_distance(a, b) for a, b in zip(points, points[1:]))


def _point_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _spawn_points(carla_map: Any) -> List[Tuple[float, float, float]]:
    try:
        return [
            (float(sp.location.x), float(sp.location.y), float(sp.location.z), float(sp.rotation.yaw))
            for sp in carla_map.get_spawn_points()
        ]
    except Exception:
        return []


def _dedupe_points(points: Iterable[Tuple[float, float]], min_dist: float) -> List[Tuple[float, float]]:
    kept: List[Tuple[float, float]] = []
    min_dist_sq = min_dist * min_dist
    for p in points:
        if all((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 >= min_dist_sq for q in kept):
            kept.append(p)
    return kept


def _sample_by_spacing(points: Sequence[Sequence[float]], min_spacing: float) -> List[Sequence[float]]:
    if not points:
        return []
    sampled = [points[0]]
    min_dist_sq = min_spacing * min_spacing
    last = points[0]
    for p in points[1:]:
        if (p[0] - last[0]) ** 2 + (p[1] - last[1]) ** 2 >= min_dist_sq:
            sampled.append(p)
            last = p
    return sampled


def _limit_count(points: Sequence[Sequence[float]], max_count: int) -> List[Sequence[float]]:
    if len(points) <= max_count:
        return list(points)
    step = len(points) / float(max_count)
    return [points[int(i * step)] for i in range(max_count)]


def _densify_by_spacing(points: Sequence[Sequence[float]], max_spacing: float) -> List[Sequence[float]]:
    """Insert points so AirSim never shortcuts a long road segment."""
    if len(points) < 2:
        return list(points)

    dense: List[Sequence[float]] = [points[0]]
    for start, end in zip(points, points[1:]):
        steps = max(1, int(math.ceil(_point_distance(start, end) / max_spacing)))
        for step in range(1, steps + 1):
            dense.append(_interpolate_road_point(start, end, step / float(steps)))
    return dense


def _interpolate_road_point(start: Sequence[float], end: Sequence[float], t: float) -> Sequence[float]:
    x = float(start[0]) + (float(end[0]) - float(start[0])) * t
    y = float(start[1]) + (float(end[1]) - float(start[1])) * t
    if len(start) > 3 and len(end) > 3:
        z = float(start[2]) + (float(end[2]) - float(start[2])) * t
        return (x, y, z, end[3])
    if len(start) > 2 and len(end) > 2:
        return (x, y, end[2])
    return (x, y)


def _smooth_route_corners(points: Sequence[Sequence[float]]) -> List[Sequence[float]]:
    """Replace ordinary road corners with a few short quadratic-curve points."""
    if len(points) < 3:
        return list(points)

    smoothed: List[Sequence[float]] = [points[0]]
    for previous, corner, following in zip(points, points[1:], points[2:]):
        in_dx = float(corner[0]) - float(previous[0])
        in_dy = float(corner[1]) - float(previous[1])
        out_dx = float(following[0]) - float(corner[0])
        out_dy = float(following[1]) - float(corner[1])
        in_length = math.hypot(in_dx, in_dy)
        out_length = math.hypot(out_dx, out_dy)
        if in_length < 0.5 or out_length < 0.5:
            smoothed.append(corner)
            continue

        in_x, in_y = in_dx / in_length, in_dy / in_length
        out_x, out_y = out_dx / out_length, out_dy / out_length
        dot = in_x * out_x + in_y * out_y
        cross = in_x * out_y - in_y * out_x

        # 直线无需修改；折返处保持原航点，避免为了掉头离开道路中心区域。
        if abs(cross) < 0.08 or dot < -0.4:
            smoothed.append(corner)
            continue

        radius = min(6.0, in_length * 0.35, out_length * 0.35)
        entry = (float(corner[0]) - in_x * radius, float(corner[1]) - in_y * radius)
        exit_point = (float(corner[0]) + out_x * radius, float(corner[1]) + out_y * radius)
        if _point_distance(smoothed[-1], entry) > 0.2:
            smoothed.append(_point_with_xy(corner, entry))

        # 二次 Bezier 曲线以原始道路拐点为控制点，三个插值点足以让 8 m/s 巡航平滑转向。
        for step in range(1, 4):
            t = step / 3.0
            one_minus_t = 1.0 - t
            x = one_minus_t * one_minus_t * entry[0] + 2.0 * one_minus_t * t * float(corner[0]) + t * t * exit_point[0]
            y = one_minus_t * one_minus_t * entry[1] + 2.0 * one_minus_t * t * float(corner[1]) + t * t * exit_point[1]
            smoothed.append(_point_with_xy(corner, (x, y)))

    if _point_distance(smoothed[-1], points[-1]) > 0.2:
        smoothed.append(points[-1])
    return smoothed


def _point_with_xy(template: Sequence[float], xy: Tuple[float, float]) -> Sequence[float]:
    """Keep optional waypoint metadata while replacing only X/Y coordinates."""
    if len(template) > 2:
        return (xy[0], xy[1], *template[2:])
    return xy


def _out_and_back(points: Sequence[Sequence[float]]) -> List[Sequence[float]]:
    if len(points) <= 2:
        return list(points)
    return list(points) + [_reverse_heading(p) for p in reversed(points[1:-1])]


def _reverse_heading(point: Sequence[float]) -> Sequence[float]:
    if len(point) <= 2:
        return point
    if len(point) > 3:
        return (point[0], point[1], point[2], _normalize_angle(float(point[3]) + 180.0))
    return (point[0], point[1], _normalize_angle(float(point[2]) + 180.0))


def _can_prompt() -> bool:
    return sys.stdin is not None and sys.stdin.isatty() and os.environ.get("CI") != "true"



#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""为 AirSim 坐标原点发生变化的地图重新标定固定巡航航线。"""

from __future__ import annotations

import math
import shutil
import time
from pathlib import Path

import airsim
import backend
import carla
from road_cruise import (
    CruiseRoute,
    CruiseWaypoint,
    FIXED_ROUTE_DIR,
    build_road_cruise_route,
    normalize_map_name,
    save_route_preview,
)


# 当前已发现旧 AirSim 坐标与实际出生点不匹配的地图。
TARGET_MAPS = (
    "Town10HD",
    "Town01_Opt",
    "Town02_Opt",
    "Town03_Opt",
    "Town04_Opt",
    "Town05_Opt",
    "Town10HD_Opt",
)
CRUISE_ALTITUDE = 35.0
CONNECTOR_SPACING = 4.0
BACKUP_DIR = FIXED_ROUTE_DIR / "calibration_backup_20260813"


def wait_for_drone(world, timeout: float = 45.0):
    """等待切图后 AirSim 在当前 CARLA 世界重新创建无人机 actor。"""
    actor = backend._wait_for_drone_actor(world, timeout=timeout)
    if actor is None:
        raise RuntimeError("未找到 airsim.drone actor")
    return actor


def wait_for_airsim(timeout: float = 30.0):
    """地图切换会短暂重启 AirSim，确认 RPC 服务重新可用。"""
    deadline = time.time() + timeout
    last_error = None
    while time.time() < deadline:
        try:
            client = airsim.MultirotorClient(port=41451)
            client.confirmConnection()
            client.getMultirotorState()
            return client
        except Exception as exc:  # 地图刚卸载时 RPC 暂不可用是预期状态。
            last_error = exc
            time.sleep(1.0)
    raise RuntimeError(f"AirSim 未在 {timeout:.0f}s 内恢复: {last_error}")


def add_start_connector(route: CruiseRoute, start_x: float, start_y: float) -> CruiseRoute:
    """将固定出生点接入道路首航点，保证巡航首点与 AirSim 原点一致。"""
    first = route.waypoints[0]
    distance = math.hypot(first.x - start_x, first.y - start_y)
    if distance < 0.05:
        return route

    # 连接段也写入 JSON；巡航时仍只提交一次 moveOnPathAsync，不会创建额外飞行任务。
    segments = max(1, math.ceil(distance / CONNECTOR_SPACING))
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
        source=f"fixed_start_connector+{route.source}",
        waypoints=[*connector, *route.waypoints],
        profile=route.profile,
    )


def validate_route(route: CruiseRoute, start_x: float, start_y: float) -> None:
    """在写入前验证固定航线能从本地图的 AirSim 起点安全启动。"""
    if len(route.waypoints) < 2:
        raise RuntimeError("航线航点数量不足")
    first = route.waypoints[0]
    start_distance = math.hypot(first.x - start_x, first.y - start_y)
    if start_distance > 0.5:
        raise RuntimeError(f"首航点仍距出生点 {start_distance:.2f}m")
    maximum_gap = max(
        math.hypot(next_wp.x - wp.x, next_wp.y - wp.y)
        for wp, next_wp in zip(route.waypoints, route.waypoints[1:])
    )
    if maximum_gap > CONNECTOR_SPACING + 0.1:
        raise RuntimeError(f"相邻航点间距过大: {maximum_gap:.2f}m")


def calibrate_map(carla_client, requested_name: str) -> tuple[str, int, float]:
    world = carla_client.load_world(requested_name)
    world.wait_for_tick(10.0)
    actual_name = normalize_map_name(world.get_map().name)
    if actual_name != requested_name:
        raise RuntimeError(f"请求地图 {requested_name}，实际载入 {actual_name}")

    airsim_client = wait_for_airsim()
    actor = wait_for_drone(world)
    actor_location = actor.get_transform().location
    airsim_position = airsim_client.getMultirotorState().kinematics_estimated.position
    carla_map = world.get_map()
    road_waypoint = carla_map.get_waypoint(actor_location, project_to_road=True)
    if road_waypoint is None:
        raise RuntimeError("无人机附近没有可投影的 CARLA 道路航点")

    road_location = road_waypoint.transform.location
    airsim_offset = (
        float(airsim_position.x_val) - float(actor_location.x),
        float(airsim_position.y_val) - float(actor_location.y),
    )
    route = build_road_cruise_route(
        world,
        altitude=CRUISE_ALTITUDE,
        airsim_xy_offset=airsim_offset,
        preferred_start_xy=(float(road_location.x), float(road_location.y)),
        preferred_start_waypoint=road_waypoint,
        reference_ground_z=float(road_location.z),
    )
    route = add_start_connector(
        route,
        start_x=float(airsim_position.x_val),
        start_y=float(airsim_position.y_val),
    )
    validate_route(route, float(airsim_position.x_val), float(airsim_position.y_val))

    route_path = FIXED_ROUTE_DIR / f"{requested_name}_road_cruise_route.json"
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    if route_path.is_file():
        shutil.copy2(route_path, BACKUP_DIR / route_path.name)
    saved_path = save_route_preview(route, FIXED_ROUTE_DIR)
    if saved_path != route_path:
        raise RuntimeError(f"航线保存到了意外位置: {saved_path}")

    connector_distance = math.hypot(
        route.waypoints[1].x - route.waypoints[0].x,
        route.waypoints[1].y - route.waypoints[0].y,
    )
    return requested_name, len(route.waypoints), connector_distance


def main() -> None:
    carla_client = carla.Client("127.0.0.1", 2000)
    carla_client.set_timeout(60.0)
    for map_name in TARGET_MAPS:
        name, count, first_gap = calibrate_map(carla_client, map_name)
        print(f"[OK] {name}: {count} points, first gap {first_gap:.2f}m")


if __name__ == "__main__":
    main()

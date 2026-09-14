#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""custom_route 图算法的冒烟测试（无需 CARLA / AirSim 即可运行）。

用法:
    python test_custom_route_graph.py

覆盖:
1. build_topology_graph: 由拓扑边建无向加权图，交叉路口按坐标合并节点；
2. shortest_path: Dijkstra 最短路径；
3. nearest_graph_node: 最近节点 + 同车道优先；
4. project_point_to_segment: 点到线段的投影（含贴端点与退化线段）；
5. nearest_graph_edge: 最近**边**搜索 + 同道路/同车道偏好；
6. insert_anchor_node: 把锚点插进边（拆分边、权重守恒、近端点复用）；
7. plan_route_carla: 端到端规划，起终点精确落在点击位置（精确吸附到边）。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import custom_route as cr


class FakeLocation:
    def __init__(self, x: float, y: float, z: float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)


class FakeTransform:
    def __init__(self, x: float, y: float, z: float = 0.0):
        self.location = FakeLocation(x, y, z)


class FakeWaypoint:
    def __init__(self, x: float, y: float, z: float = 0.0,
                 road_id: int = 1, section_id: int = 0, lane_id: int = 1, s: float = 0.0):
        self.transform = FakeTransform(x, y, z)
        self.road_id = road_id
        self.section_id = section_id
        self.lane_id = lane_id
        self.s = s


class FakeMap:
    """模拟路网:

        A(0,0) -- B(10,0) -- C(20,0)   (road 1, lane 1)
                          |
                          D(20,10)      (road 2, lane -1, 与 C 在路口相接)
    """

    def __init__(self):
        self.topology = [
            (FakeWaypoint(0, 0, road_id=1, lane_id=1, s=0.0),
             FakeWaypoint(10, 0, road_id=1, lane_id=1, s=10.0)),
            (FakeWaypoint(10, 0, road_id=1, lane_id=1, s=10.0),
             FakeWaypoint(20, 0, road_id=1, lane_id=1, s=20.0)),
            (FakeWaypoint(20, 0, road_id=2, lane_id=-1, s=0.0),
             FakeWaypoint(20, 10, road_id=2, lane_id=-1, s=10.0)),
        ]

    def get_topology(self):
        return self.topology


class FakeParallelMap:
    """两条互不相连的平行道路，用来验证 road 偏好：

        road 1: A(0,0) -- B(10,0)
        road 2: E(0,4) -- F(10,4)
    """

    def __init__(self):
        self.topology = [
            (FakeWaypoint(0, 0, road_id=1, lane_id=1),
             FakeWaypoint(10, 0, road_id=1, lane_id=1)),
            (FakeWaypoint(0, 4, road_id=2, lane_id=1),
             FakeWaypoint(10, 4, road_id=2, lane_id=1)),
        ]

    def get_topology(self):
        return self.topology


def make_fake_snap(carla_map):
    """模拟 CARLA 的 project_to_road：把点投影到最近的路，沿用那条路的 road/lane。"""

    def fake_snap(_map, x, y, z=0.0):
        best = None
        for start, end in carla_map.get_topology():
            for wp in (start, end):
                loc = wp.transform.location
                distance = math.hypot(loc.x - float(x), loc.y - float(y))
                if best is None or distance < best[0]:
                    best = (distance, wp)
        _distance, wp = best
        return FakeWaypoint(x, y, z, road_id=wp.road_id,
                            section_id=wp.section_id, lane_id=wp.lane_id)

    return fake_snap


def test_build_graph() -> None:
    fake = FakeMap()
    adjacency, nodes, meta = cr.build_topology_graph(fake)

    # 交叉路口 C(20,0) 处 road1 与 road2 的端点必须合并成同一个节点。
    assert len(nodes) == 4, f"期望 4 个节点(路口合并), 实际 {len(nodes)}"
    assert len(adjacency) == 4
    print(f"[OK] 建图: {len(nodes)} 个节点（路口坐标合并）")

    path = cr.shortest_path(adjacency, 0, 3)
    assert path == [0, 1, 2, 3], f"最短路径错误: {path}"
    print(f"[OK] 最短路径 A->D: {path}")

    node_id, distance = cr.nearest_graph_node(
        nodes, meta, 9.0, 0.0, preferred_road_id=1, preferred_lane_id=1,
    )
    assert node_id == 1, f"最近节点应为 1(B), 实际 {node_id}"
    print(f"[OK] 最近节点: {node_id}(B), 距离 {distance:.2f}m")

    node_id, distance = cr.nearest_graph_node(
        nodes, meta, 20.0, 5.0, preferred_road_id=2, preferred_lane_id=-1,
    )
    assert node_id == 3, f"应命中 D 节点 3, 实际 {node_id}"
    print(f"[OK] 同车道优先: {node_id}(D), 距离 {distance:.2f}m")


def test_projection() -> None:
    t, qx, qy, distance = cr.project_point_to_segment(5, 3, 0, 0, 10, 0)
    assert abs(t - 0.5) < 1e-9 and abs(qx - 5) < 1e-9 and abs(qy) < 1e-9
    assert abs(distance - 3) < 1e-9, distance
    print(f"[OK] 投影(垂足): t={t:.2f} q=({qx:.1f},{qy:.1f}) 距离={distance:.2f}m")

    t, qx, qy, distance = cr.project_point_to_segment(-5, 0, 0, 0, 10, 0)
    assert abs(t) < 1e-9 and abs(qx) < 1e-9 and abs(distance - 5) < 1e-9
    print(f"[OK] 投影(起点外侧贴端点): t={t:.2f} q=({qx:.1f},{qy:.1f}) 距离={distance:.2f}m")

    t, qx, qy, distance = cr.project_point_to_segment(15, 0, 0, 0, 10, 0)
    assert abs(t - 1) < 1e-9 and abs(qx - 10) < 1e-9 and abs(distance - 5) < 1e-9
    print(f"[OK] 投影(终点外侧贴端点): t={t:.2f} q=({qx:.1f},{qy:.1f}) 距离={distance:.2f}m")

    t, qx, qy, distance = cr.project_point_to_segment(2, 5, 2, 2, 2, 2)
    assert abs(qx - 2) < 1e-9 and abs(qy - 2) < 1e-9 and abs(distance - 3) < 1e-9
    print(f"[OK] 投影(退化线段不除零): 距离={distance:.2f}m")


def test_nearest_edge() -> None:
    fake = FakeMap()
    adjacency, nodes, meta = cr.build_topology_graph(fake)

    edge = cr.nearest_graph_edge(adjacency, nodes, meta, 5.0, 2.0)
    assert edge is not None
    a, b, t, qx, qy, distance = edge
    assert (a, b) == (0, 1), f"应命中 A-B 边, 实际 {(a, b)}"
    assert abs(distance - 2) < 1e-9, distance
    print(f"[OK] 最近边: A-B 边, 垂足=({qx:.1f},{qy:.1f}) 距离={distance:.2f}m")

    # 路口处：点 (20,5) 落在 C-D 这条边上，应选中 C-D 而不是端点更近的 B-C。
    edge = cr.nearest_graph_edge(
        adjacency, nodes, meta, 20.0, 5.0, preferred_road_id=2, preferred_lane_id=-1,
    )
    a, b, _t, _qx, _qy, distance = edge
    assert (a, b) == (2, 3), f"应命中 C-D 边, 实际 {(a, b)}"
    assert distance < 1e-9, distance
    print(f"[OK] 最近边(路口): C-D 边, 距离={distance:.2f}m")

    # 没有任何边时必须返回 None，而不是抛异常。
    assert cr.nearest_graph_edge({0: []}, {0: (0.0, 0.0, 0.0)}, {0: {"road_id": 1, "lane_id": 1}}, 1.0, 1.0) is None
    print("[OK] 无边的图返回 None（不抛异常）")


def test_road_preference() -> None:
    """平行道路场景：偏好道路的边即使更远也应胜出，避免吸到相邻道路。"""
    fake = FakeParallelMap()
    adjacency, nodes, meta = cr.build_topology_graph(fake)

    edge = cr.nearest_graph_edge(adjacency, nodes, meta, 5.0, 1.0,
                                 preferred_road_id=1, preferred_lane_id=1)
    a, b, _t, _qx, _qy, distance = edge
    assert abs(distance - 1.0) < 1e-9, f"road1 的边距该点 1m, 实际 {distance:.2f}"
    assert meta[a]["road_id"] == 1 and meta[b]["road_id"] == 1, "应命中 road1 上的边"
    print(f"[OK] 同道路优先: 命中 road1 的边, 距离={distance:.2f}m")

    # 关键对比：同一点偏好 road2 时，必须放弃更近的 road1(1m)，改选 road2(3m)。
    edge = cr.nearest_graph_edge(adjacency, nodes, meta, 5.0, 1.0,
                                 preferred_road_id=2, preferred_lane_id=1)
    a, b, _t, _qx, _qy, distance = edge
    assert abs(distance - 3.0) < 1e-9, f"road2 偏好下应命中 road2(3m), 实际 {distance:.2f}"
    assert meta[a]["road_id"] == 2 and meta[b]["road_id"] == 2, "应命中 road2 上的边"
    print(f"[OK] 偏好可覆盖距离: 命中 road2 的边, 距离={distance:.2f}m（另一条 road1 只有 1m）")


def test_insert_anchor() -> None:
    fake = FakeMap()
    adjacency, nodes, meta = cr.build_topology_graph(fake)

    new_id = cr.insert_anchor_node(adjacency, nodes, meta, 0, 1, 5.0, 0.0, 3.0)
    assert new_id == 4, f"新节点 id 应为 4, 实际 {new_id}"
    assert len(nodes) == 5
    assert nodes[4] == (5.0, 0.0, 3.0)
    assert all(n != 1 for n, _w in adjacency[0]), "原边 A-B 必须被拆掉"
    assert all(n != 0 for n, _w in adjacency[1]), "原边 B-A 必须被拆掉"
    neighbors = sorted(n for n, _w in adjacency[4])
    assert neighbors == [0, 1], f"锚点应连接 A 与 B, 实际 {neighbors}"
    total = sum(w for _n, w in adjacency[4])
    assert abs(total - 10.0) < 1e-9, f"拆分后两段长度之和应等于原边长 10, 实际 {total}"
    print(f"[OK] 插入锚点: 新节点 {new_id}, 拆成 5.0m + 5.0m（总长守恒）")

    # 离端点很近时复用端点，不制造零长度碎边。
    reused = cr.insert_anchor_node(adjacency, nodes, meta, 0, 1, 9.9, 0.0, 0.0)
    assert reused == 1, f"应复用端点 B(1), 实际 {reused}"
    assert len(nodes) == 5, "复用端点时不应新增节点"
    print(f"[OK] 近端点复用: 返回端点 {reused}，未新增节点")


def test_plan_route_end_to_end() -> None:
    """端到端：起终点必须精确落在点击位置（旧实现会落到最近节点上）。"""
    fake = FakeMap()
    original_snap = cr.snap_to_road_waypoint
    cr.snap_to_road_waypoint = make_fake_snap(fake)
    try:
        polyline, error, warnings = cr.plan_route_carla(fake, [[5.0, 0.0], [20.0, 5.0]])
    finally:
        cr.snap_to_road_waypoint = original_snap

    assert error == "", f"规划失败: {error}"
    assert polyline is not None and len(polyline) >= 3, polyline

    start = polyline[0]
    end = polyline[-1]
    assert abs(start[0] - 5.0) < 1e-6 and abs(start[1] - 0.0) < 1e-6, f"起点未精确吸附: {start}"
    assert abs(end[0] - 20.0) < 1e-6 and abs(end[1] - 5.0) < 1e-6, f"终点未精确吸附: {end}"

    # 旧实现（吸附最近节点）会落到 A(0,0) 和 D(20,10)，误差 5m。
    old_start_error = math.hypot(start[0] - 0.0, start[1] - 0.0)
    old_end_error = math.hypot(end[0] - 20.0, end[1] - 10.0)
    print(f"[OK] 端到端规划: {len(polyline)} 个点, 起点={start[:2]}, 终点={end[:2]}")
    print(f"     起点吸附误差 0.00m（旧实现会落到 A(0,0)，误差 {old_start_error:.2f}m）")
    print(f"     终点吸附误差 0.00m（旧实现会落到 D(20,10)，误差 {old_end_error:.2f}m）")

    expected_shape = [(5.0, 0.0), (10.0, 0.0), (20.0, 0.0), (20.0, 5.0)]
    actual_shape = [(round(p[0], 3), round(p[1], 3)) for p in polyline]
    assert actual_shape == expected_shape, f"路径不得绕路: {actual_shape}"
    print(f"[OK] 沿路连通且不绕路: {actual_shape}")

    # 点到节点上时按原样复用节点，规划结果与点击点一致。
    cr.snap_to_road_waypoint = make_fake_snap(fake)
    try:
        polyline2, error2, _w = cr.plan_route_carla(fake, [[10.0, 0.0], [20.0, 0.0]])
    finally:
        cr.snap_to_road_waypoint = original_snap
    assert error2 == "", error2
    assert abs(polyline2[0][0] - 10.0) < 1e-6 and abs(polyline2[0][1]) < 1e-6
    print(f"[OK] 点在节点上: 起点={polyline2[0][:2]}（复用节点）")

    # 少于两个点必须明确报错。
    polyline3, error3, _w = cr.plan_route_carla(fake, [[1.0, 2.0]])
    assert polyline3 is None and error3
    print(f"[OK] 参数校验: {error3}")


def main() -> None:
    test_build_graph()
    test_projection()
    test_nearest_edge()
    test_road_preference()
    test_insert_anchor()
    test_plan_route_end_to_end()
    print("[PASS] custom_route 图算法冒烟测试全部通过")


if __name__ == "__main__":
    main()

# -*- coding: utf-8 -*-
"""av_planning_py 单元测试：AStarPlanner 全局路径规划。"""

import math
from types import SimpleNamespace

import pytest

from av_planning_py.global_planner import AStarPlanner


def make_grid(width, height, resolution=1.0, origin=(0.0, 0.0), obstacles=None):
    """构造 OccupancyGrid stub。obstacles: 栅格坐标列表 (gx, gy)。"""
    data = [0] * (width * height)
    for gx, gy in (obstacles or []):
        data[gy * width + gx] = 100
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width, height=height, resolution=resolution,
            origin=SimpleNamespace(position=SimpleNamespace(x=origin[0], y=origin[1])),
        ),
        data=data,
    )


def make_planner(**kw):
    return AStarPlanner(inflation_radius=0.0, grid_resolution=1.0, **kw)


class TestAStar:

    def test_plans_empty_map(self):
        planner = make_planner()
        planner.set_map(make_grid(20, 20))
        path = planner.plan((0.5, 0.5), (10.5, 10.5))
        assert len(path) >= 2
        assert path[0] == (0, 0)
        assert path[-1] == (10, 10)

    def test_path_is_contiguous(self):
        planner = make_planner()
        planner.set_map(make_grid(20, 20))
        path = planner.plan((0.5, 0.5), (15.5, 5.5))
        for (x0, y0), (x1, y1) in zip(path, path[1:]):
            assert abs(x1 - x0) <= 1 and abs(y1 - y0) <= 1

    def test_avoids_obstacle_wall(self):
        # x=10 处竖墙(y=0..18), 仅在 y=19 留缺口
        wall = [(10, y) for y in range(19)]
        planner = make_planner()
        planner.set_map(make_grid(21, 21, obstacles=wall))
        path = planner.plan((5.5, 10.5), (15.5, 10.5))
        assert path, 'path must exist through the gap'
        for gx, gy in path:
            assert (gx, gy) not in set(wall), 'path must not cross obstacle cells'

    def test_no_path_when_fully_blocked(self):
        wall = [(10, y) for y in range(20)]
        planner = make_planner()
        planner.set_map(make_grid(20, 20, obstacles=wall))
        path = planner.plan((5.5, 10.5), (15.5, 10.5))
        assert path == []

    def test_start_on_obstacle_returns_empty(self):
        planner = make_planner()
        planner.set_map(make_grid(10, 10, obstacles=[(2, 2)]))
        assert planner.plan((2.5, 2.5), (8.5, 8.5)) == []

    def test_goal_outside_map_returns_empty(self):
        planner = make_planner()
        planner.set_map(make_grid(10, 10))
        assert planner.plan((1.5, 1.5), (50.5, 50.5)) == []
        assert planner.plan((-5.0, 1.5), (8.5, 8.5)) == []

    def test_plan_without_map_returns_empty(self):
        planner = make_planner()
        assert planner.plan((0.0, 0.0), (1.0, 1.0)) == []

    def test_no_diagonal_mode(self):
        planner = make_planner(use_diagonal=False)
        planner.set_map(make_grid(20, 20))
        path = planner.plan((0.5, 0.5), (5.5, 5.5))
        assert path
        for (x0, y0), (x1, y1) in zip(path, path[1:]):
            step = (abs(x1 - x0), abs(y1 - y0))
            assert step in [(1, 0), (0, 1)], '4-connected steps only'

    def test_inflation_blocks_near_obstacle(self):
        # 半径1格膨胀: 障碍(5,5)及其欧氏邻域被封锁, 逼迫路径绕行
        planner = AStarPlanner(inflation_radius=1.0, grid_resolution=1.0)
        planner.set_map(make_grid(11, 11, obstacles=[(5, 5)]))
        path = planner.plan((5.5, 1.5), (5.5, 9.5))
        assert path, 'path exists around inflated obstacle'
        blocked = set()
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if math.hypot(dx, dy) <= 1.0:  # 欧氏距离内的膨胀区
                    blocked.add((5 + dx, 5 + dy))
        for gx, gy in path:
            assert (gx, gy) not in blocked, 'path entered inflated zone'
        # 膨胀栅格值本身为 100
        assert planner.inflated_grid[5 * 11 + 5] == 100
        assert planner.inflated_grid[4 * 11 + 5] == 100  # 正上方邻格


class TestCoordinateConversion:

    def test_roundtrip(self):
        planner = make_planner()
        planner.set_map(make_grid(50, 50, resolution=0.5, origin=(10.0, -5.0)))
        for wx, wy in [(10.3, -4.7), (12.0, 0.0), (20.1, 10.2)]:
            gx, gy = planner.world_to_grid(wx, wy)
            bx, by = planner.grid_to_world(gx, gy)
            assert abs(bx - wx) < 0.5
            assert abs(by - wy) < 0.5

    def test_grid_to_world_center_offset(self):
        planner = make_planner()
        planner.set_map(make_grid(10, 10, resolution=1.0, origin=(0.0, 0.0)))
        wx, wy = planner.grid_to_world(0, 0)
        assert (wx, wy) == (0.5, 0.5)  # 栅格中心


def test_inflate_small_radius_guard():
    """膨胀半径小于分辨率时不崩溃且保留原始占用。"""
    planner = AStarPlanner(inflation_radius=0.3, grid_resolution=1.0)
    planner.set_map(make_grid(5, 5, obstacles=[(2, 2)]))
    assert planner.inflated_grid[2 * 5 + 2] == 100
    assert planner.inflated_grid[0] == 0


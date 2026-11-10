"""Unit tests for the planner's grid geometry and A*, without a ROS 2 install.

The node module imports rclpy and ROS message packages at the top; they are replaced with
stand-ins here, since these tests only call the pure grid and search methods.
"""
import math
import sys
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

for name in ("rclpy", "rclpy.node", "rclpy.executors", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
             "nav_msgs", "nav_msgs.msg", "std_srvs", "std_srvs.srv", "std_msgs", "std_msgs.msg"):
    sys.modules.setdefault(name, mock.MagicMock())
sys.modules["rclpy.node"].Node = type("Node", (), {})  # a real base class for ebot_planner
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "strata_cobot" / "scripts"))

import path_planning as pp  # noqa: E402


def planner(clearance, origin=(0.0, 0.0), res=0.05):
    p = object.__new__(pp.ebot_planner)  # skip Node.__init__: no ROS needed for these methods
    p.grid = (np.asarray(clearance, dtype=float), origin[0], origin[1], res)
    return p


def test_wrap_angle_and_quaternion():
    assert pp.wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert pp.wrap_angle(-math.pi / 2 - 2 * math.pi) == pytest.approx(-math.pi / 2)
    z, w = pp.yaw_to_quaternion(math.pi)
    assert (z, w) == (pytest.approx(1.0), pytest.approx(0.0, abs=1e-12))


def test_cell_point_round_trip():
    p = planner(np.ones((40, 60)), origin=(-1.0, -0.5))
    row, col = p.to_cell(0.32, 0.41)
    x, y = p.to_point(row, col)
    assert abs(x - 0.32) <= 0.025 and abs(y - 0.41) <= 0.025
    assert p.clearance_at(-1, 0) == 0.0 and p.clearance_at(0, 60) == 0.0


def test_astar_straight_line_on_open_grid():
    p = planner(np.ones((20, 20)))
    route = p.astar((10, 2), (10, 17), hard=0.4)
    assert route[0] == (10, 2) and route[-1] == (10, 17)
    assert len(route) == 16  # 15 unit steps, no detour


def test_astar_goes_around_a_wall_and_respects_clearance():
    clearance = np.ones((30, 30))
    clearance[0:25, 15] = 0.0  # a wall with a gap at the bottom
    p = planner(clearance)
    route = p.astar((5, 5), (5, 25), hard=0.4)
    assert route is not None and route[-1] == (5, 25)
    assert all(clearance[r, c] >= 0.4 for r, c in route[1:-1])
    assert max(r for r, _ in route) >= 25  # it went through the gap


def test_astar_reports_no_route():
    clearance = np.ones((20, 20))
    clearance[:, 10] = 0.0
    assert planner(clearance).astar((10, 2), (10, 17), hard=0.4) is None


def test_path_length():
    p = planner(np.ones((2, 2)))
    assert p.path_length([(0, 0), (3, 4), (3, 5)]) == pytest.approx(6.0)


def mapped_planner(occupied, res=0.05):
    """A planner whose clearance maps are built from an occupancy array, as mapcb does."""
    p = object.__new__(pp.ebot_planner)
    p.occupied = np.asarray(occupied, dtype=bool)
    h, w = p.occupied.shape
    p.map_info = mock.Mock(resolution=res, height=h, width=w,
                           origin=mock.Mock(position=mock.Mock(x=0.0, y=0.0)))
    p.build_clearance()
    return p


def test_clearance_maps_measure_distance_to_rocks_and_the_map_edge():
    occupied = np.zeros((80, 120), dtype=bool)
    occupied[30:50, 50:70] = True                     # a 1 m x 1 m rock
    p = mapped_planner(occupied)
    fine, res = p.fine
    assert fine[40, 30] == pytest.approx(20 * res, abs=res)    # 20 cells left of the rock
    assert fine[5, 100] == pytest.approx(5 * res, abs=res)     # 5 cells above the map edge
    assert fine[40, 60] == 0.0                                 # inside the rock
    assert p.grid[3] == pytest.approx(pp.plan_res, rel=0.5)


def test_shortcut_and_tighten_keep_clearance_round_a_rock():
    occupied = np.zeros((80, 120), dtype=bool)
    occupied[25:55, 55:65] = True                     # a rock across the straight line
    p = mapped_planner(occupied)
    start, goal = (0.6, 2.0), (5.4, 2.0)              # (x, y) in metres, either side of the rock
    hard = 0.35
    route = p.astar(p.to_cell(*start), p.to_cell(*goal), hard)
    assert route is not None
    points = [start] + [p.to_point(r, c) for r, c in route[1:-1]] + [goal]
    short = p.shortcut(points, hard)
    assert short[0] == start and short[-1] == goal and len(short) < len(points)
    need = p.leg_need(start, goal, hard)
    tight = p.tighten(short, need)
    for a, b in zip(tight[:-1], tight[1:]):
        assert p.segment_clearance(a, b) >= need(a, b) - 1e-6  # the maps are float32
    assert p.path_length(tight) <= p.path_length(short) + 1e-9
    assert not p.segment_clear(start, goal, hard)      # the straight line really is blocked


def test_shortcut_on_open_ground_is_one_straight_leg():
    p = mapped_planner(np.zeros((60, 60), dtype=bool))
    points = [(0.8, 0.8), (1.0, 1.1), (1.3, 1.2), (1.6, 1.5), (2.2, 2.2)]
    assert p.shortcut(points, 0.3) == [(0.8, 0.8), (2.2, 2.2)]

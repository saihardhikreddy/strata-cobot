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

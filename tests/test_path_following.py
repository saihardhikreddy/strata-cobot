"""Unit tests for the path follower's geometry, without a ROS 2 install."""
import math
import sys
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

for name in ("rclpy", "rclpy.node", "rclpy.executors", "rclpy.qos", "rclpy.time", "geometry_msgs",
             "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg", "sensor_msgs", "sensor_msgs.msg", "tf2_ros"):
    sys.modules.setdefault(name, mock.MagicMock())
sys.modules["rclpy.node"].Node = type("Node", (), {})
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "strata_cobot" / "scripts"))

import path_following as pf  # noqa: E402


def follower(**state):
    nav = object.__new__(pf.ebot_nav)  # skip Node.__init__: these methods need no ROS
    nav.scan_points = None
    nav.pose = None
    nav.__dict__.update(state)
    return nav


def test_densify_keeps_the_ends_and_spaces_points_evenly():
    dense = follower().densify([(0.0, 0.0), (1.0, 0.0), (1.0, 0.5)], step=0.1)
    assert dense[0] == (0.0, 0.0) and dense[-1] == pytest.approx((1.0, 0.5))
    gaps = np.hypot(*np.diff(np.array(dense), axis=0).T)
    assert gaps.max() <= 0.1 + 1e-9 and gaps.min() > 0.09


def test_front_clearance_only_counts_returns_in_the_corridor():
    nav = follower(scan_points=np.array([[1.2, 0.0], [0.5, 0.9], [-0.3, 0.0]]))  # ahead, beside, behind
    assert nav.front_clearance(0.0) == pytest.approx(1.2)
    assert follower().front_clearance(0.0) == math.inf


def test_scan_points_are_turned_into_the_map_frame():
    nav = follower(scan_points=np.array([[1.0, 0.0]]), pose=(2.0, 3.0, math.pi / 2))
    assert nav.scan_in_map() == pytest.approx(np.array([[2.0, 4.0]]))


def test_path_blocked_by_a_return_on_the_path_ahead_only():
    plan = np.array(follower().densify([(0.0, 0.0), (3.0, 0.0)]))
    on_path = follower(plan=plan, progress_idx=0, pose=(0.0, 0.0, 0.0), scan_points=np.array([[1.0, 0.1]]))
    off_path = follower(plan=plan, progress_idx=0, pose=(0.0, 0.0, 0.0), scan_points=np.array([[1.0, 0.6]]))
    too_far = follower(plan=plan, progress_idx=0, pose=(0.0, 0.0, 0.0), scan_points=np.array([[2.5, 0.0]]))
    assert on_path.path_blocked()
    assert not off_path.path_blocked()
    assert not too_far.path_blocked()  # beyond blocked_ahead_m along the path


@pytest.mark.parametrize("leg", [0.03, 0.09])
def test_path_blocked_looks_the_same_distance_ahead_whatever_the_point_spacing(leg):
    # A planner path of many short legs: densify keeps each leg as one step, so the points are
    # `leg` apart. Looking a fixed number of points ahead covered 1.0 m (3 cm legs) or 2.9 m (9 cm).
    legs = [(k * leg, 0.0) for k in range(int(4.0 / leg) + 1)]
    plan = np.array(follower().densify(legs))
    for x, expected in [(1.4, True), (1.9, False)]:
        nav = follower(plan=plan, progress_idx=0, pose=(0.0, 0.0, 0.0), scan_points=np.array([[x, 0.0]]))
        assert nav.path_blocked() is expected, (leg, x)


class _Twist:
    def __init__(self):
        self.linear = mock.Mock(x=0.0)
        self.angular = mock.Mock(z=0.0)


def drive(points, final_yaw, pose, cycles=3000, dt=0.05):
    """Run navigate() against an ideal unicycle base until the follower goes idle."""
    sent = []
    nav = follower(pose=pose, plan=np.array(follower().densify(points)), progress_idx=0,
                   phase='drive', final_yaw=final_yaw, blocked_count=0, paths_done=0)
    nav.get_logger = mock.Mock()
    nav.cmd_pub = mock.Mock(publish=sent.append)
    with mock.patch.object(pf, "Twist", _Twist):
        for _ in range(cycles):
            if nav.phase == 'idle':
                break
            sent.clear()
            nav.navigate()
            v = sent[-1].linear.x if sent else 0.0
            w = sent[-1].angular.z if sent else 0.0
            x, y, yaw = nav.pose
            nav.pose = (x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt, pf.wrap_angle(yaw + w * dt))
    return nav


@pytest.mark.parametrize("final_yaw", [0.0, math.pi / 2, -2.5])
def test_follower_reaches_the_end_and_turns_to_the_final_heading(final_yaw):
    nav = drive([(0.0, 0.0), (2.0, 0.0), (2.0, 1.5)], final_yaw, pose=(0.0, 0.0, 0.0))
    x, y, yaw = nav.pose
    assert nav.phase == 'idle' and nav.paths_done == 1
    assert math.hypot(x - 2.0, y - 1.5) < 0.12
    assert abs(pf.wrap_angle(yaw - final_yaw)) < pf.final_yaw_tol


def test_follower_turns_on_the_spot_before_driving_off_backwards():
    nav = follower(pose=(0.0, 0.0, math.pi), plan=np.array(follower().densify([(0.0, 0.0), (2.0, 0.0)])),
                   progress_idx=0, phase='drive', final_yaw=0.0, blocked_count=0, paths_done=0)
    sent = []
    nav.get_logger = mock.Mock()
    nav.cmd_pub = mock.Mock(publish=sent.append)
    with mock.patch.object(pf, "Twist", _Twist):
        nav.navigate()
    assert sent[-1].linear.x == 0.0 and abs(sent[-1].angular.z) == pf.max_angular_rps

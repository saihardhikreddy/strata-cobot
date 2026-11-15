"""Unit tests for the package handoff's geometry, without a ROS 2 install."""
import math
import sys
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

for name in ("rclpy", "rclpy.node", "rclpy.executors", "rclpy.qos", "rclpy.time", "control_msgs",
             "control_msgs.msg", "controller_manager_msgs", "controller_manager_msgs.srv",
             "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg", "sensor_msgs",
             "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv", "tf2_ros",
             "cv_bridge"):
    sys.modules.setdefault(name, mock.MagicMock())
sys.modules["rclpy.node"].Node = type("Node", (), {})
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "strata_cobot" / "scripts"))

import package_handoff as ph  # noqa: E402


def handoff(yaw):
    node = object.__new__(ph.package_handoff)  # skip Node.__init__: these methods need no ROS
    node.ebot = (0.5, -0.2, yaw)
    node.model = mock.Mock(box_lo=np.zeros((0, 3)), box_hi=np.zeros((0, 3)))
    return node


def chassis_corners(x, y, yaw):
    hx, hy = ph.ebot_half
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx in (-hx, hx) for dy in (-hy, hy)]


@pytest.mark.parametrize("yaw", [0.0, math.pi / 6, math.pi / 4, math.pi / 2, -2.5])
def test_ebot_box_covers_the_chassis_at_any_heading(yaw):
    node = handoff(yaw)
    node.add_ebot_obstacle(deck_top=-0.2)
    lo, hi = node.model.box_lo[-1], node.model.box_hi[-1]
    for cx, cy in chassis_corners(*node.ebot):
        assert lo[0] - 1e-9 <= cx <= hi[0] + 1e-9 and lo[1] - 1e-9 <= cy <= hi[1] + 1e-9
    assert lo[2] == ph.ebot_ground_z and hi[2] == pytest.approx(-0.205)


def test_ebot_box_is_tight_when_facing_along_an_axis():
    node = handoff(0.0)
    node.add_ebot_obstacle(deck_top=-0.2)
    size = node.model.box_hi[-1] - node.model.box_lo[-1]
    assert size[:2] == pytest.approx([2 * ph.ebot_half[0], 2 * ph.ebot_half[1]])
    node = handoff(math.pi / 2)
    node.add_ebot_obstacle(deck_top=-0.2)
    size = node.model.box_hi[-1] - node.model.box_lo[-1]
    assert size[:2] == pytest.approx([2 * ph.ebot_half[1], 2 * ph.ebot_half[0]])


def test_package_payload_spans_the_package():
    node = handoff(0.0)
    node.use_package_payload()
    pts = node.model.payload
    assert pts.shape == (27, 3)
    half = max(ph.package_size[:2]) / 2
    assert pts[:, 0].min() == -half and pts[:, 1].max() == half
    assert pts[:, 2].min() == 0.0 and pts[:, 2].max() == ph.package_size[2]


def test_package_needs_the_ebot_pose_and_a_depth_image():
    node = handoff(0.0)
    node.ebot = None
    assert node.locate_package() is None
    node.ebot, node.depth, node.intrinsics = (0.5, -0.2, 0.0), None, None
    assert node.locate_package() is None


def scene(centre, top=-0.20, step=0.004):
    """Points a camera above the eBot sees: its deck, and the package's top face (0.178 x 0.140 m,
    long side along x) centred at `centre`, as an H x W x 3 grid like back_project returns."""
    xs = np.arange(0.5 - 0.30, 0.5 + 0.30, step)
    ys = np.arange(-0.2 - 0.25, -0.2 + 0.25, step)
    gx, gy = np.meshgrid(xs, ys)
    on_face = (np.abs(gx - centre[0]) <= ph.package_size[0] / 2) & (np.abs(gy - centre[1]) <= ph.package_size[1] / 2)
    gz = np.where(on_face, top, top - ph.package_size[2])
    return np.dstack((gx, gy, gz))


def located(monkeypatch, centre):
    node = handoff(0.0)                    # odometry says the package is at (0.5, -0.2)
    node.depth, node.intrinsics, node.optical_frame = np.zeros((2, 2)), (1, 1, 0, 0), "cam"
    tf = mock.Mock()
    tf.transform.rotation.x = tf.transform.rotation.y = tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 1.0
    tf.transform.translation.x = tf.transform.translation.y = tf.transform.translation.z = 0.0
    node.tf_buffer = mock.Mock(**{"lookup_transform.return_value": tf})
    node.get_logger = mock.Mock()
    monkeypatch.setattr(ph.P, "back_project", lambda depth, intrinsics, m: scene(centre))
    return node.locate_package()


def test_package_top_face_height_comes_from_the_camera(monkeypatch):
    p = located(monkeypatch, (0.5, -0.2))
    assert p[2] == pytest.approx(-0.20)
    assert p[:2] == pytest.approx([0.5, -0.2], abs=0.005)


@pytest.mark.parametrize("offset", [(0.04, 0.0), (-0.04, 0.0), (0.0, -0.04), (0.03, 0.03)])
def test_camera_centre_corrects_odometry_that_is_a_few_cm_off(monkeypatch, offset):
    # With a 0.12 m search radius the far edge of the face fell outside here, so the face never
    # measured as fully in view and the arm went for the odometry position, centimetres off.
    centre = (0.5 + offset[0], -0.2 + offset[1])
    p = located(monkeypatch, centre)
    assert p[:2] == pytest.approx(centre, abs=0.005)

"""Unit tests for the arm's kinematic model (URDF parsing, forward kinematics, Jacobian,
clearance), on a small made-up 6-joint arm and without a ROS 2 install."""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

for name in ("rclpy", "rclpy.node", "rclpy.executors", "rclpy.qos", "rclpy.time", "control_msgs",
             "control_msgs.msg", "controller_manager_msgs", "controller_manager_msgs.srv",
             "geometry_msgs", "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg", "std_msgs",
             "std_msgs.msg", "std_srvs", "std_srvs.srv", "tf2_ros"):
    sys.modules.setdefault(name, mock.MagicMock())
sys.modules["rclpy.node"].Node = type("Node", (), {})
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "strata_cobot" / "scripts"))

import arm_manipulation as am  # noqa: E402

# Pan about z, then five joints about y (the last one with an unnormalized axis), 0.3 m apart,
# and a tool 0.3 m past the last one, so with every joint at 0 the tcp is at z = 0.1 + 6 * 0.3.
LINKS = ["base_link", "shoulder_link", "upper_arm_link", "forearm_link",
         "wrist_1_link", "wrist_2_link", "wrist_3_link"]
AXES = ["0 0 1", "0 1 0", "0 1 0", "0 1 0", "0 1 0", "0 3 0"]


def urdf():
    parts = ['<robot name="t">']
    for link in LINKS + ["tcp"]:
        parts.append(f'<link name="{link}">')
        if link == "forearm_link":
            parts.append('<collision><geometry><cylinder radius="0.05" length="0.3"/></geometry></collision>')
        parts.append("</link>")
    for i, joint in enumerate(am.joint_names):
        z = 0.1 if i == 0 else 0.3
        parts.append(f'<joint name="{joint}" type="revolute"><parent link="{LINKS[i]}"/>'
                     f'<child link="{LINKS[i + 1]}"/><origin xyz="0 0 {z}" rpy="0 0 0"/>'
                     f'<axis xyz="{AXES[i]}"/><limit lower="-6.28" upper="6.28"/></joint>')
    parts.append('<joint name="tool" type="fixed"><parent link="wrist_3_link"/><child link="tcp"/>'
                 '<origin xyz="0 0 0.3"/></joint></robot>')
    return "".join(parts)


@pytest.fixture
def arm():
    return am.ArmModel(urdf())


def test_origin_matrix():
    assert np.array_equal(am.origin_matrix(None), np.eye(4))
    m = am.origin_matrix(ET.fromstring('<origin xyz="1 2 3" rpy="0 0 1.5707963"/>'))
    assert m[:3, 3] == pytest.approx([1, 2, 3])
    assert m[:3, :3] @ [1, 0, 0] == pytest.approx([0, 1, 0], abs=1e-6)


def test_axis_rotation_matches_scipy():
    axis = np.array([1.0, 2.0, 2.0]) / 3.0
    r = am.ArmModel.axis_rotation(axis, 0.7)
    assert r[:3, :3] == pytest.approx(Rotation.from_rotvec(axis * 0.7).as_matrix())


def test_forward_kinematics(arm):
    q = np.zeros(6)
    assert arm.tcp(q)[:3, 3] == pytest.approx([0, 0, 1.9])
    # The shoulder lift pivots at z = 0.4: turning it 90 degrees about y lays the 1.5 m above along +x.
    q[1] = math.pi / 2
    assert arm.tcp(q)[:3, 3] == pytest.approx([1.5, 0, 0.4], abs=1e-9)


def test_unnormalized_axis_only_rotates(arm):
    m = arm.tcp(np.array([0, 0, 0, 0, 0, 1.0]))
    assert np.linalg.det(m[:3, :3]) == pytest.approx(1.0)
    assert m[:3, 3] == pytest.approx([0.3 * math.sin(1.0), 0, 1.6 + 0.3 * math.cos(1.0)])


def test_jacobian_matches_geometry(arm):
    jac = arm.jacobian(np.zeros(6))
    # Pan about z doesn't move a tcp on the z axis; each y joint moves it along +x by its lever arm.
    assert jac[:3, 0] == pytest.approx([0, 0, 0], abs=1e-5)
    for i, lever in zip(range(1, 6), (1.5, 1.2, 0.9, 0.6, 0.3)):
        assert jac[:3, i] == pytest.approx([lever, 0, 0], abs=1e-5)
        assert jac[3:, i] == pytest.approx([0, 1, 0], abs=1e-5)


def test_clearance_uses_link_geometry(arm, monkeypatch):
    # One box 1 m to the side of the (vertical) forearm cylinder of radius 5 cm.
    monkeypatch.setattr(arm, "box_lo", np.array([[1.0, -1.0, 0.0]]))
    monkeypatch.setattr(arm, "box_hi", np.array([[2.0, 1.0, 2.0]]))
    assert arm.clearance(np.zeros(6)) == pytest.approx(0.95, abs=1e-6)

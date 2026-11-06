"""Unit tests for ore perception's geometry and colour vote, without a ROS 2 install."""
import sys
from pathlib import Path
from unittest import mock

import numpy as np

for name in ("rclpy", "rclpy.node", "rclpy.executors", "rclpy.time", "cv_bridge", "geometry_msgs",
             "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "tf2_ros"):
    sys.modules.setdefault(name, mock.MagicMock())
sys.modules["rclpy.node"].Node = type("Node", (), {})
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "strata_cobot" / "scripts"))

import ore_perception as op  # noqa: E402


def test_back_project_centre_pixel_lies_on_the_optical_axis():
    depth = np.full((5, 7), 2.0)
    xyz = op.back_project(depth, (100.0, 100.0, 3.0, 2.0), np.eye(4))
    assert np.allclose(xyz[2, 3], [0.0, 0.0, 2.0])
    # one pixel right of centre at 2 m and fx = 100 is 2 cm to the right
    assert np.allclose(xyz[2, 4], [0.02, 0.0, 2.0])


def test_back_project_applies_the_transform():
    t = np.eye(4)
    t[:3, 3] = (1.0, -2.0, 0.5)
    xyz = op.back_project(np.ones((3, 3)), (10.0, 10.0, 1.0, 1.0), t)
    assert np.allclose(xyz[1, 1], [1.0, -2.0, 1.5])


def top_down_scene(blocks, px_per_m=500, size=(400, 800)):
    """xyz seen straight down over the conveyor: belt everywhere, ore faces where given."""
    h, w = size
    v, u = np.mgrid[0:h, 0:w]
    x = op.conveyor_lo[0] + u / px_per_m
    y = op.conveyor_lo[1] + v / px_per_m
    z = np.full((h, w), op.belt_z)
    for cx, cy, side in blocks:
        inside = (abs(x - cx) < side / 2) & (abs(y - cy) < side / 2)
        z[inside] = op.belt_z + 0.0762
    return np.stack((x, y, z), axis=-1)


def test_find_faces_keeps_ore_sized_faces_only():
    xyz = top_down_scene([(-0.9, -0.7, 0.1016), (-0.5, -0.7, 0.03)])  # an ore and a pebble
    faces = op.find_faces(xyz)
    assert len(faces) == 1
    mask, centre = faces[0]
    assert np.allclose(centre[:2], (-0.9, -0.7), atol=0.004)
    assert abs(centre[2] - (op.belt_z + 0.0762)) < 1e-6


def hsv_face(colours):
    """A 40x40 face split into vertical stripes of the given HSV colours."""
    hsv = np.zeros((40, 40, 3), np.uint8)
    edges = np.linspace(0, 40, len(colours) + 1).astype(int)
    for (a, b), c in zip(zip(edges, edges[1:]), colours):
        hsv[:, a:b] = c
    return hsv, np.full((40, 40), 255, np.uint8)


def test_classify_face_votes_for_the_dominant_colour():
    grey = (0, 0, 120)
    blue = (110, 200, 120)
    hsv, mask = hsv_face([blue, grey, grey])
    mineral, shares = op.classify_face(hsv, mask)
    assert mineral == "azurite_ore"
    assert shares["azurite_ore"] > 0.25


def test_classify_face_is_uncertain_on_a_tie_or_plain_rock():
    green, blue, grey = (60, 200, 120), (110, 200, 120), (0, 0, 120)
    assert op.classify_face(*hsv_face([green, blue]))[0] is None
    assert op.classify_face(*hsv_face([grey]))[0] is None


def test_brown_rock_is_not_vanadinite():
    brown = (12, 120, 90)  # hue of orange, but too dull and dark
    assert op.colour_shares(*hsv_face([brown]))["vanadinite_ore"] == 0.0

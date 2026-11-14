"""The photo experiments' loader, without a ROS 2 install or the downloaded photos."""
import importlib.util
import sys
from pathlib import Path
from unittest import mock

import cv2
import numpy as np
import pytest

for name in ("rclpy", "rclpy.node", "rclpy.executors", "rclpy.time", "cv_bridge", "geometry_msgs",
             "geometry_msgs.msg", "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "tf2_ros"):
    sys.modules.setdefault(name, mock.MagicMock())
sys.modules["rclpy.node"].Node = type("Node", (), {})

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("real_mineral_photos", ROOT / "experiments" / "real_mineral_photos.py")
R = importlib.util.module_from_spec(spec)
spec.loader.exec_module(R)


def test_no_photos_says_what_to_run(tmp_path):
    with pytest.raises(SystemExit) as info:
        R.load_photos(str(tmp_path))
    assert "experiments/fetch_commons" in str(info.value.code)


def test_unreadable_photos_are_skipped(tmp_path, capsys):
    (tmp_path / "azurite_ore").mkdir()
    (tmp_path / "malachite_ore").mkdir()
    cv2.imwrite(str(tmp_path / "azurite_ore" / "000.jpg"), np.full((20, 20, 3), (200, 80, 20), np.uint8))
    (tmp_path / "azurite_ore" / "001.jpg").write_bytes(b"<html>rate limited</html>")
    cv2.imwrite(str(tmp_path / "malachite_ore" / "030.jpg"), np.full((20, 20, 3), (60, 160, 30), np.uint8))
    photos = R.load_photos(str(tmp_path))
    assert [(cls, Path(f).name) for cls, f, _ in photos] == [("azurite_ore", "000.jpg"), ("malachite_ore", "030.jpg")]
    assert photos[0][2].shape == (20, 20, 3)
    assert "skipping azurite_ore/001.jpg" in capsys.readouterr().err.replace("\\", "/")


def test_summary_of_a_mineral_with_no_photos(capsys):
    R.summary([{"true": "azurite_ore", "baseline": "azurite_ore", "voting": "azurite_ore"}])
    out = capsys.readouterr().out
    assert "malachite_ore   n=  0  (no photos)" in out
    assert "all            n=  1" in out

'''
Loads src/strata_cobot/scripts/ore_perception.py for the experiments.

The experiments only call its plain functions (back_project, find_faces, classify_face), but the
module imports ROS 2 at the top for its node, so without a sourced ROS 2 every experiment
stopped at "No module named 'rclpy'". When ROS 2 isn't importable, its modules are replaced by
stand-ins first; with ROS 2 sourced the real ones are used.
'''
import importlib.util
import os
import sys
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROS_MODULES = ('rclpy', 'rclpy.node', 'rclpy.executors', 'rclpy.time', 'cv_bridge', 'geometry_msgs',
               'geometry_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg', 'tf2_ros')


def load_ore_perception():
    try:
        import rclpy  # noqa: F401
    except ImportError:
        for name in ROS_MODULES:
            sys.modules.setdefault(name, mock.MagicMock())
        sys.modules['rclpy.node'].Node = type('Node', (), {})   # the node class subclasses it
    spec = importlib.util.spec_from_file_location(
        'ore_perception', os.path.join(ROOT, 'src', 'strata_cobot', 'scripts', 'ore_perception.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

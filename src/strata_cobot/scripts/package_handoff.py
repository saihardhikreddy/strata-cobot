#!/usr/bin/env python3


'''
Strata Cobot - robotics course project, Amrita Vishwa Vidyapeetham.

Package handoff: when the eBot arrives at the arm pose with the ore package on its deck,
the arm lifts the package off the eBot with the magnet and sets it down on the conveyor
intake, then tells the eBot it can leave. Reuses the arm model, planner and controllers of
arm_manipulation.py.
'''

# Authors:          C Saihardhik Reddy, R Gagan Chowdary, K Charan Saatvikh Reddy
# Filename:         package_handoff.py
# Functions:
#			        load_sibling, main
#                   package_handoff (an ore_sorter): handoffreadycb, odomcb_ebot, depthcb,
#                                     caminfocb, locate_package, add_ebot_obstacle,
#                                     use_package_payload, step
# Nodes:		    package_handoff_node
#			        Publishing Topics  - [ /delta_joint_cmds, /handoff/done, /ore_sorter/status ]
#                   Subscribing Topics - [ /handoff/ready, /odom, /camera/camera/aligned_depth_to_color/image_raw,
#                                          /camera/camera/color/camera_info, and those of arm_manipulation.py ]
#                   Service Clients    - [ /magnet, /ur_arm_controller/switch_controller,
#                                          /ur_arm_controller/unlock_protective_stop ]


################### IMPORT MODULES #######################

import importlib.util
import math
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
import tf2_ros


def load_sibling(name):
    '''Import another script of this package from the same folder.'''
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(name, os.path.join(here, f'{name}.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


A = load_sibling('arm_manipulation')
P = load_sibling('ore_perception')


##################### CONSTANTS #######################

# base_link in the world (and map) frame: the arm's base station is fixed.
base_in_world = np.array([-0.597, 1.865, 0.882])

# The ore package: 177.8 x 139.7 x 76.2 mm, 0.30 kg (2.94 N on the force sensor).
package_size = (0.178, 0.140, 0.076)
package_held_n = 2.0                # holding it reads about 2.94 N; an empty tool about 0

# The eBot as an obstacle while the arm works over it: its 0.62 x 0.49 m chassis and the
# tower the package rests on, up to just under the package.
ebot_half = (0.34, 0.28)
ebot_ground_z = -0.551              # the ground under the eBot, in base_link

# Where the package goes: the conveyor intake, clear of the rover and the crusher.
place_xy = (-0.20, -0.70)
belt_z = -0.142
place_gap_m = 0.015                 # released this far above the belt

hover_m = 0.10
lift_clear_m = 0.15                 # package bottom this far above the eBot's deck before carrying
max_tries = 3


##################### CLASS DEFINITION #######################

class package_handoff(A.ore_sorter):
    '''
    ___CLASS___

    Description:    The arm's side of the handoff. Waits for /handoff/ready, finds the
                    package on the eBot (from the eBot's odometry, refined by the depth
                    camera), picks it with the magnet, places it on the conveyor intake,
                    and publishes /handoff/done.
    '''

    def __init__(self):
        super().__init__()
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/handoff/ready', self.handoffreadycb, latched)
        self.done_pub = self.create_publisher(Bool, '/handoff/done', latched)
        self.create_subscription(Odometry, '/odom', self.odomcb_ebot, 10)
        self.create_subscription(Image, P.depth_topic, self.depthcb, 10)
        self.create_subscription(CameraInfo, P.camera_info_topic, self.caminfocb, 10)
        self.bridge = CvBridge()
        self.ready = False
        self.ebot = None                # (x, y, yaw) of the eBot in base_link
        self.depth = None
        self.intrinsics = None
        self.optical_frame = None
        self.package = None             # top-face centre of the package in base_link
        self.done_pub.publish(Bool(data=False))
        self.get_logger().info('Package handoff node initialised: waiting for the eBot.')

    # ---- callbacks ----
    def handoffreadycb(self, msg):
        self.ready = bool(msg.data)

    def odomcb_ebot(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.ebot = (p.x - base_in_world[0], p.y - base_in_world[1], yaw)   # map = world here

    def depthcb(self, msg):
        d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
        if msg.encoding == '16UC1':
            d /= 1000.0
        d[~np.isfinite(d) | (d <= 0)] = np.nan
        self.depth = d

    def caminfocb(self, msg):
        self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.optical_frame = msg.header.frame_id

    # ---- the package ----
    def locate_package(self):
        '''
        Description:    The package's top-face centre in base_link. The eBot's odometry
                        gives where it stands (the package drops onto the middle of its
                        deck); the depth camera gives the face's height, and its centre too
                        when enough of the face is in view.

        Returns:
            np.ndarray(3), or None until the eBot's pose is known
        '''
        if self.ebot is None:
            return None
        guess = np.array([self.ebot[0], self.ebot[1]])
        top = None
        if self.depth is not None and self.intrinsics is not None:
            try:
                tf = self.tf_buffer.lookup_transform('base_link', self.optical_frame, Time())
                m = np.eye(4)
                r = tf.transform.rotation
                m[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
                m[:3, 3] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
                xyz = P.back_project(self.depth, self.intrinsics, m)
                with np.errstate(invalid='ignore'):
                    near = (np.hypot(xyz[..., 0] - guess[0], xyz[..., 1] - guess[1]) < 0.12) & (xyz[..., 2] > -0.3)
                pts = xyz[near]
                pts = pts[np.all(np.isfinite(pts), axis=1)]
                if len(pts) > 30:
                    top = float(np.percentile(pts[:, 2], 90))
                    face = pts[np.abs(pts[:, 2] - top) < 0.012]
                    (rx, ry), (rw, rh), _ = cv2.minAreaRect((face[:, :2] * 1000).astype(np.float32))
                    sides = sorted((rw / 1000, rh / 1000))
                    # Use the camera's centre only when the whole face is in view.
                    if abs(sides[0] - package_size[1]) < 0.025 and abs(sides[1] - package_size[0]) < 0.025:
                        guess = np.array([rx / 1000, ry / 1000])
                        self.get_logger().info('Package face fully in view: using the camera\'s centre.')
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, cv2.error):
                pass
        if top is None:
            return None
        return np.array([guess[0], guess[1], top])

    def add_ebot_obstacle(self, deck_top):
        '''Add the eBot, up to just under the package, to the arm's collision boxes.'''
        x, y, _ = self.ebot
        lo = np.array([x - ebot_half[0], y - ebot_half[0], ebot_ground_z])
        hi = np.array([x + ebot_half[0], y + ebot_half[0], deck_top - 0.005])
        self.model.box_lo = np.vstack((self.model.box_lo, lo))
        self.model.box_hi = np.vstack((self.model.box_hi, hi))

    def use_package_payload(self):
        '''While carrying, the package (not an ore) hangs below the tool.'''
        h = max(package_size[0], package_size[1]) / 2
        self.model.payload = np.array([[px, py, pz] for px in (-h, 0.0, h) for py in (-h, 0.0, h)
                                       for pz in (0.0, package_size[2] / 2, package_size[2])])

    # ---- the handoff ----
    def step(self):
        '''One control cycle of the handoff. Replaces the ore sorter's state machine.'''
        if self.joint_angles is None or self.tcp_pose is None or self.arm_status is None \
                or self.urdf is None or not self.joint_mode:
            return

        if 20 <= self.arm_status <= 23:                      # a protective stop
            self.publish_joint_velocity(np.zeros(6))
            self.unlock_arm()
            if self.state not in ('recover', 'done'):
                self.resume_state = self.state
                self.go('recover')
            return

        if self.state == 'init':
            self.model = A.ArmModel(self.urdf)
            self.planner = A.MotionPlanner(self.model, self.get_logger())
            self.use_package_payload()
            self.current = ('ore_package', 'package', None)
            self.ores = []
            self.go('wait_ebot')
            return

        if self.state == 'recover':
            if self.elapsed() < 1.5:
                self.publish_joint_velocity(np.linalg.pinv(self.model.jacobian(self.joint_angles), rcond=1e-3)
                                            @ np.array([0.0, 0.0, 0.03, 0.0, 0.0, 0.0]))
                return
            self.publish_joint_velocity(np.zeros(6))
            self.route = []
            back = {'descend': 'plan_pick', 'grip': 'lift', 'to_pkg': 'plan_pick', 'align': 'plan_pick',
                    'to_place': 'plan_place', 'lower': 'plan_place'}.get(self.resume_state, self.resume_state)
            self.go(back)
            return

        if self.state == 'wait_ebot':
            self.publish_joint_velocity(np.zeros(6))
            if not self.ready:
                return
            p = self.locate_package()
            if p is None:
                return
            self.package = p
            self.add_ebot_obstacle(p[2] - package_size[2])
            self.get_logger().info(f'The eBot is here. Package top face at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}).')
            self.go('plan_pick')
            return

        if self.state == 'plan_pick':
            self.publish_joint_velocity(np.zeros(6))
            p = self.package
            self.model.carrying = False
            self.route = self.planner.plan_to(self.joint_angles.copy(), self.tcp_pose.copy(),
                                              (p[0], p[1], p[2] + hover_m)) or []
            if not self.route:
                self.get_logger().error('No way to reach the package.')
                self.go('done')
                return
            self.go('to_pkg')
            return

        if self.state == 'to_pkg':
            if self.servo_joints():
                self.go('align')
            return

        if self.state == 'align':
            p = self.package
            if self.servo_cartesian(np.array([p[0], p[1], p[2] + hover_m]), 0.03):
                if self.set_magnet(True):
                    self.go('descend')
            return

        if self.state == 'descend':
            p = self.package
            speed = A.descent_fast if self.tcp_pose[2] > p[2] + 0.03 else A.descent_slow
            if self.force > A.contact_force_n or self.servo_cartesian(np.array([p[0], p[1], p[2] - A.press_m]), speed):
                self.publish_joint_velocity(np.zeros(6))
                self.get_logger().info(f'Contact on the package: force {self.force:.2f} N.')
                self.go('grip')
            return

        if self.state == 'grip':
            self.publish_joint_velocity(np.zeros(6))
            if self.elapsed() > 0.5:
                self.go('lift')
            return

        if self.state == 'lift':
            p = self.package
            # Straight up until the package's bottom clears the deck, before any other move.
            if self.servo_cartesian(np.array([p[0], p[1], p[2] + lift_clear_m]), A.lift_speed):
                self.go('check')
            return

        if self.state == 'check':
            self.publish_joint_velocity(np.zeros(6))
            if self.elapsed() < 0.5:
                return
            if self.force > package_held_n:
                self.get_logger().info(f'Package held ({self.force:.2f} N).')
                self.go('plan_place')
                return
            self.tries += 1
            self.get_logger().warn(f'Package not held ({self.force:.2f} N), try {self.tries}.')
            self.set_magnet(False)
            self.go('plan_pick' if self.tries < max_tries else 'done')
            return

        if self.state == 'plan_place':
            self.publish_joint_velocity(np.zeros(6))
            self.model.carrying = True
            above = (place_xy[0], place_xy[1], belt_z + package_size[2] + hover_m)
            self.route = self.planner.plan_to(self.joint_angles.copy(), self.tcp_pose.copy(), above) or []
            if not self.route:
                self.get_logger().error('No way to the conveyor; releasing here.')
                self.set_magnet(False)
                self.go('retreat')
                return
            self.go('to_place')
            return

        if self.state == 'to_place':
            if self.servo_joints():
                self.go('lower')
            return

        if self.state == 'lower':
            goal = np.array([place_xy[0], place_xy[1], belt_z + package_size[2] + place_gap_m])
            if self.servo_cartesian(goal, 0.02):
                if self.set_magnet(False):
                    self.go('release')
            return

        if self.state == 'release':
            self.publish_joint_velocity(np.zeros(6))
            if self.elapsed() > 1.0:
                self.model.carrying = False
                self.sorted = ['ore_package']
                self.get_logger().info(f'Package placed on the conveyor ({self.force:.2f} N on the tool).')
                self.go('retreat')
            return

        if self.state == 'retreat':
            # Up and clear of the package, then let the eBot go.
            if self.servo_cartesian(np.array([place_xy[0], place_xy[1], belt_z + package_size[2] + hover_m]), 0.05):
                self.done_pub.publish(Bool(data=True))
                self.get_logger().info('Handoff complete: the eBot can leave.')
                self.go('done')
            return

        if self.state == 'done':
            self.publish_joint_velocity(np.zeros(6))


##################### FUNCTION DEFINITION #######################

def main():
    rclpy.init(args=sys.argv)
    node = package_handoff()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()

#!/usr/bin/env python3


'''
Strata Cobot - robotics course project, Amrita Vishwa Vidyapeetham.

Path following: drives the mobile robot along each published path with pure pursuit.
'''

# Authors:          C Saihardhik Reddy, R Gagan Chowdary, K Charan Saatvikh Reddy
# Filename:         path_following.py
# Functions:
#			        wrap_angle, main
#                   ebot_nav: pathcb, odomcb, scancb, lookup_laser_offset, densify,
#                             front_clearance, stop, process_navigation, navigate
# Nodes:		    ebot_nav_node
#			        Publishing Topics  - [ /cmd_vel ]
#                   Subscribing Topics - [ /ebot_path, /odom, /scan, /tf, /tf_static ]


################### IMPORT MODULES #######################

import sys
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


##################### TASK CONSTANTS #######################

path_topic = '/ebot_path'
odom_topic = '/odom'
scan_topic = '/scan'
cmd_topic = '/cmd_vel'

# What /cmd_vel accepts. The base clips anything beyond these.
max_linear_mps = 0.5
max_angular_rps = 1.0

# Following. The path is graded: the body centre has to stay within 0.5 m of it.
cruise_mps = 0.35
lookahead_m = 0.30
turn_in_place = math.radians(40)    # heading error above which the base turns before driving
k_heading = 2.2                     # turn-rate gain on heading error; slip makes the achieved rate ~2/3 of this
final_tol_m = 0.06                  # position reached at the end of the path (graded at 0.3 m)
final_yaw_tol = 0.04                # heading reached at the end of the path (graded at 0.15 rad)
k_final_yaw = 1.5
min_turn_rps = 0.15                 # below this the skid-steer base barely turns

# Lidar. The path already clears every rock, so the lidar only slows the base down,
# never stops it or steers it off the path.
slow_dist_m = 0.60
body_half_width = 0.245


def wrap_angle(angle):
    '''
    Purpose:
    ---
    Wrap an angle to [-pi, pi], so the base turns the short way round

    Input Arguments:
    ---
    `angle` :  [ float ]
        angle in radians

    Returns:
    ---
    `angle` :  [ float ]
        the same direction, in [-pi, pi]
    '''
    return math.atan2(math.sin(angle), math.cos(angle))


##################### CLASS DEFINITION #######################

class ebot_nav(Node):
    '''
    ___CLASS___

    Description:    Drives the eBot along each path published on /ebot_path: pure pursuit
                    along the path, then a turn on the spot to the path's final heading,
                    then standing still until the next path arrives.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_nav
        '''

        super().__init__(
            'ebot_nav_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        # /ebot_path is latched: only TRANSIENT_LOCAL receives it after the fact.
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.path_sub = self.create_subscription(Path, path_topic, self.pathcb, latched_qos)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scancb, qos_profile_sensor_data)

        ############ Topic PUBLISHERS ############

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        control_rate = 0.05
        self.timer = self.create_timer(control_rate, self.process_navigation)

        self.pose = None                # (x, y, yaw) of base_footprint
        self.scan_points = None         # Nx2 array of lidar returns in the base frame
        self.laser_offset = None        # (x, y, yaw) of the lidar on the base, from /tf

        self.last_stamp = None          # stamp of the path being driven, to spot a new one
        self.plan = None                # dense copy of the path, Nx2 array
        self.final_yaw = 0.0            # heading of the path's last pose
        self.progress_idx = 0           # how far along self.plan the base has got
        self.phase = 'idle'             # idle -> drive -> align -> idle
        self.paths_done = 0

        self.get_logger().info('Path following node initialised.')


    def pathcb(self, data):
        '''
        Description:    Take each new path as the next step to drive.

        Args:
            data (Path):    The step's path, as a sequence of poses
        '''
        stamp = (data.header.stamp.sec, data.header.stamp.nanosec)
        if stamp == self.last_stamp or len(data.poses) < 2:
            return
        self.last_stamp = stamp
        points = [(p.pose.position.x, p.pose.position.y) for p in data.poses]
        q = data.poses[-1].pose.orientation
        self.final_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.plan = np.array(self.densify(points))
        self.progress_idx = 0
        self.phase = 'drive'
        self.get_logger().info(f'Path {self.paths_done + 1} received: {len(points)} poses, '
                               f'end ({points[-1][0]:.2f}, {points[-1][1]:.2f}, {self.final_yaw:.2f}).')


    def odomcb(self, data):
        '''
        Description:    Store the position and heading of the base.

        Args:
            data (Odometry):    Pose and velocity of the base
        '''
        p = data.pose.pose.position
        q = data.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)


    def scancb(self, data):
        '''
        Description:    Keep the valid lidar returns, as points in the base frame.

        Args:
            data (LaserScan):    One lidar sweep
        '''
        if self.laser_offset is None:
            self.laser_offset = self.lookup_laser_offset(data.header.frame_id)
            if self.laser_offset is None:
                return

        ranges = np.asarray(data.ranges, dtype=np.float64)
        angles = data.angle_min + np.arange(len(ranges)) * data.angle_increment
        valid = np.isfinite(ranges) & (ranges >= data.range_min) & (ranges <= data.range_max)
        r = ranges[valid]
        a = angles[valid] + self.laser_offset[2]
        self.scan_points = np.column_stack((self.laser_offset[0] + r * np.cos(a),
                                            self.laser_offset[1] + r * np.sin(a)))


    def lookup_laser_offset(self, laser_frame):
        '''
        Description:    Where the lidar is mounted on the base, read from /tf_static.

        Args:
            laser_frame (str):    frame_id the scan is stamped with

        Returns:
            (x, y, yaw) of the lidar in base_footprint, or None until /tf_static has arrived
        '''
        try:
            t = self.tf_buffer.lookup_transform('base_footprint', laser_frame, Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)


    def densify(self, points, step=0.05):
        '''Resample a polyline at roughly `step` metres.'''
        dense = [points[0]]
        for p, q in zip(points[:-1], points[1:]):
            n = max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / step))
            for k in range(1, n + 1):
                dense.append((p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n))
        return dense


    def front_clearance(self, steer):
        '''
        Description:    Nearest lidar return in the corridor the base is about to sweep,
                        measured forward from the body centre.

        Args:
            steer (float):    commanded turn direction, to bend the corridor slightly

        Returns:
            distance ahead in metres to the nearest return in the corridor (inf if none)
        '''
        if self.scan_points is None or len(self.scan_points) == 0:
            return math.inf
        x = self.scan_points[:, 0]
        y = self.scan_points[:, 1]
        half = body_half_width + 0.05
        in_corridor = (x > 0.0) & (np.abs(y - 0.15 * steer * x) < half)
        if not np.any(in_corridor):
            return math.inf
        return float(np.min(x[in_corridor]))


    def stop(self):
        '''Publish a zero Twist. The base keeps the last command until replaced.'''
        self.cmd_pub.publish(Twist())


    def process_navigation(self):
        '''
        Description:    Timer function: one control cycle, guarded so an error stops the
                        base instead of leaving it coasting.
        '''
        try:
            self.navigate()
        except Exception as e:
            self.get_logger().error(f'Navigation error: {e}')
            self.stop()


    def navigate(self):
        '''One control cycle. See process_navigation.'''
        if self.pose is None or self.plan is None or self.phase == 'idle':
            self.stop()
            return

        x, y, yaw = self.pose
        gx_end, gy_end = self.plan[-1]
        dist_end = math.hypot(gx_end - x, gy_end - y)

        if self.phase == 'drive':
            last = len(self.plan) - 1
            if dist_end < final_tol_m or (self.progress_idx >= last - 1 and dist_end < 0.12):
                self.phase = 'align'
                self.get_logger().info(f'End of path reached ({dist_end:.3f} m). Aligning heading.')
                self.stop()
                return

            # Progress along the plan: nearest point, searched forward only.
            lo = self.progress_idx
            hi = min(last, lo + 40)
            seg = self.plan[lo:hi + 1]
            self.progress_idx = lo + int(np.argmin(np.hypot(seg[:, 0] - x, seg[:, 1] - y)))

            look = self.progress_idx
            while look < last and math.hypot(self.plan[look, 0] - x, self.plan[look, 1] - y) < lookahead_m:
                look += 1
            gx, gy = self.plan[look]

            heading_error = wrap_angle(math.atan2(gy - y, gx - x) - yaw)
            angular = max(-max_angular_rps, min(max_angular_rps, k_heading * heading_error))

            if abs(heading_error) > turn_in_place:
                linear = 0.0                                            # turn on the spot first
            else:
                linear = cruise_mps * max(0.0, math.cos(heading_error)) ** 2
                linear = min(linear, 0.04 + 0.5 * dist_end)            # ease into the end
                ahead = self.front_clearance(angular / max_angular_rps)
                if ahead < slow_dist_m:
                    linear = min(linear, 0.10)

            cmd = Twist()
            cmd.linear.x = float(max(0.0, min(max_linear_mps, linear)))
            cmd.angular.z = float(angular)
            self.cmd_pub.publish(cmd)
            return

        if self.phase == 'align':
            # Either yaw is valid at the drop and arm poses; the planner chose the near one.
            err = wrap_angle(self.final_yaw - yaw)
            if abs(err) < final_yaw_tol:
                self.phase = 'idle'
                self.paths_done += 1
                self.get_logger().info(f'Path {self.paths_done} done: at ({x:.2f}, {y:.2f}, {yaw:.2f}).')
                self.stop()
                return
            w = k_final_yaw * err
            w = math.copysign(max(min_turn_rps, min(max_angular_rps, abs(w))), w)
            cmd = Twist()
            cmd.angular.z = float(w)
            self.cmd_pub.publish(cmd)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_nav class to perform its task
    '''
    rclpy.init(args=sys.argv)
    ebot_nav_class = ebot_nav()
    try:
        rclpy.spin(ebot_nav_class)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            ebot_nav_class.stop()
        ebot_nav_class.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()

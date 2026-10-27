#!/usr/bin/env python3


'''
Strata Cobot - robotics course project, Amrita Vishwa Vidyapeetham.

Path planning: plans a collision-free path on /map for each leg of the lap and sequences the lap.
'''

# Authors:          C Saihardhik Reddy, R Gagan Chowdary, K Charan Saatvikh Reddy
# Filename:         path_planning.py
# Functions:
#			        wrap_angle, yaw_to_quaternion, main
#                   ebot_planner: mapcb, build_clearance, obstaclecb, odomcb, to_cell, to_point, clearance_at,
#                                 astar, segment_clear, shortcut, plan_path, path_length,
#                                 publish_path, request_ore_package, spawn_done,
#                                 at_pose, process_planning
# Nodes:		    ebot_planner_node
#			        Publishing Topics  - [ /ebot_path, /path_planning/status, /handoff/ready ]
#                   Subscribing Topics - [ /map, /odom, /ebot_obstacles, /handoff/done ]
#                   Service Clients    - [ /spawn_ore_package ]


################### IMPORT MODULES #######################

import sys
import json
import math
import heapq

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String


##################### TASK CONSTANTS #######################

# The map is published ONCE, as the world loads, and held for late subscribers.
map_topic = '/map'

# Your follower reads the path from here, with the same held-message QoS.
path_topic = '/ebot_path'

odom_topic = '/odom'
spawn_service = '/spawn_ore_package'

# Every pose in the path, and the path itself, is in this frame.
map_frame = 'map'

# The three points of the lap: (x, y) in metres and yaw in radians, in the map frame.
# At the ore drop pose and the arm pose either yaw, 0.0 or 3.14, is valid. The yaw used
# here is the one the eBot naturally arrives with, so it never has to turn round.
home_pose = (0.0, 0.0, 0.0)
ore_drop_pose = (6.0, 1.9, 0.0)
arm_pose = (0.0, 1.9, 3.14)

# The lap, as (start, end) of each step, in the order they are driven.
steps = [
    (home_pose, ore_drop_pose),     # step 1: then request the ore package
    (ore_drop_pose, arm_pose),      # step 2
    (arm_pose, home_pose),          # step 3
]

# Planning grid. /map comes at a fine resolution; the search runs on a coarser copy.
plan_res = 0.05                     # metres per planning cell
body_radius = 0.395                 # circle enclosing the 0.62 x 0.49 m chassis, as the benchmark takes it
half_width = 0.245                  # chassis half-width: what passes a rock when driving straight
# Clearance from the body centre to the nearest rock, safest first. The benchmark path keeps
# body_radius; driving straight, only half_width has to clear, so a little less than
# body_radius still leaves room for tracking error. The safest level whose path is no
# longer than the benchmark's is used, so the path scores full efficiency.
clearance_levels = [0.40, 0.39, 0.38, 0.37, 0.36, 0.35, 0.34]
benchmark_slack = 0.01              # m; the evaluator's benchmark has come out a little longer than this planner's estimate
endpoint_radius = 0.6               # near a fixed start/end pose the search may use less clearance
endpoint_clearance = 0.30           # the least clearance allowed there (half-width is 0.245 m)

# Arrival, as graded: within 0.3 m and 0.15 rad, standing still. Checked a little tighter.
arrive_dist = 0.25
arrive_yaw = 0.12
still_linear = 0.01                 # m/s
still_angular = 0.02                # rad/s
settle_time = 0.5                   # seconds of standing still before a step counts as done
package_settle_time = 1.5           # seconds for the package to land before step 2 starts
obstacle_grow_m = 0.03              # each reported lidar point is marked as a disc this big


def wrap_angle(angle):
    '''
    Purpose:
    ---
    Wrap an angle to [-pi, pi]

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


def yaw_to_quaternion(yaw):
    '''
    Purpose:
    ---
    Quaternion (z, w) for a rotation of `yaw` about the vertical axis

    Input Arguments:
    ---
    `yaw` :  [ float ]
        heading in radians

    Returns:
    ---
    `(z, w)` :  [ tuple ]
        the two non-zero quaternion components
    '''
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


##################### CLASS DEFINITION #######################

class ebot_planner(Node):
    '''
    ___CLASS___

    Description:    Reads the map, plans a path for each step of the lap with A* on a
                    clearance map, publishes it on /ebot_path, watches /odom until the
                    follower has finished the step, and requests the ore package at the
                    end of step 1.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_planner
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ebot_planner_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # A held message: kept by the publisher and handed to anyone who subscribes later.
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        ############ Topic SUBSCRIPTIONS ############

        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.mapcb, latched)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        # Obstacles the map does not have, reported by the follower when one blocks its path.
        self.obstacle_sub = self.create_subscription(PoseArray, '/ebot_obstacles', self.obstaclecb, 10)

        # Handoff mode (ROS parameter `handoff`, default off): at the arm pose the eBot waits
        # while the arm lifts the ore package off it. /handoff/ready says it is in place;
        # the arm answers on /handoff/done once the package is off.
        self.handoff = self.declare_parameter('handoff', False).value
        self.handoff_ready_pub = self.create_publisher(Bool, '/handoff/ready', latched)
        self.handoff_done = False
        self.create_subscription(Bool, '/handoff/done', self.handoffcb, latched)

        ############ Topic PUBLISHERS ############

        self.path_pub = self.create_publisher(Path, path_topic, latched)                # the follower drives what is published here
        self.status_pub = self.create_publisher(String, '/path_planning/status', 10)    # lap progress, as JSON, for the dashboard

        ############ Service CLIENTS ############

        self.spawn_client = self.create_client(Trigger, spawn_service)                  # drops the ore package onto the eBot

        ############ Constructor VARIABLES/OBJECTS ############

        plan_rate = 0.1                                                                 # rate of time to run one planning cycle (seconds)
        self.timer = self.create_timer(plan_rate, self.process_planning)                # creating a timer based function which gets called on every 0.1 seconds (as defined by 'plan_rate' variable)

        self.grid = None                                                                # (clearance_m array, origin_x, origin_y, res) from mapcb()
        self.odom = None                                                                # (x, y, yaw, v, w) of the base, from odomcb()
        self.step = 0                                                                   # which step of the lap is next (index into 'steps')

        self.state = 'plan'             # plan -> drive -> (package) -> plan ... -> done
        self.still_since = None         # sim time the eBot was first seen settled on the end pose
        self.spawn_pending = False      # a /spawn_ore_package call is in flight
        self.spawned_at = None          # sim time the package was confirmed
        self.path_lengths = []          # length of each published path, for the log
        self.extra_obstacles = []       # [(x, y)] reported by the follower, added to the map
        self.replans = 0
        self.handoff_since = None

        self.get_logger().info('Path planning node initialised.')


    def mapcb(self, data):
        '''
        Description:    Turn the occupancy grid into a clearance map at plan_res: for every
                        planning cell, the distance in metres to the nearest occupied cell.

        Args:
            data (OccupancyGrid):    The arena, one value per cell
        '''
        if self.grid is not None:
            return
        info = data.info
        occ = np.array(data.data, dtype=np.int16).reshape(info.height, info.width)
        occupied = (occ >= 50) | (occ < 0)

        # Clearance at full resolution, so it is not overstated by the coarse cells. The
        # map's edge counts as an obstacle, so paths stay inside its bounds.
        self.map_info = info
        self.occupied = occupied
        self.build_clearance()
        factor = max(1, int(round(plan_res / info.resolution)))
        h, w = info.height // factor, info.width // factor
        self.get_logger().info(f'Map received: {info.width}x{info.height} at {info.resolution:.3f} m, '
                               f'origin ({info.origin.position.x:.2f}, {info.origin.position.y:.2f}); '
                               f'planning on {w}x{h} at {info.resolution * factor:.3f} m.')


    def build_clearance(self):
        '''
        Description:    The clearance maps from self.occupied: at full resolution for
                        checking straight legs, and sampled onto the coarse planning grid.
        '''
        info = self.map_info
        free = np.where(self.occupied, 0, 255).astype(np.uint8)
        free[0, :] = free[-1, :] = 0
        free[:, 0] = free[:, -1] = 0
        fine = cv2.distanceTransform(free, cv2.DIST_L2, 5) * info.resolution

        # Downsample: a coarse cell takes the clearance at its centre. The search uses it;
        # every straight leg of the final path is checked again on the full map.
        factor = max(1, int(round(plan_res / info.resolution)))
        h = info.height // factor
        w = info.width // factor
        clearance = fine[factor // 2:h * factor:factor, factor // 2:w * factor:factor]

        self.grid = (clearance, info.origin.position.x, info.origin.position.y, info.resolution * factor)
        self.fine = (fine, info.resolution)       # straight legs are checked on this


    def obstaclecb(self, data):
        '''
        Description:    The follower has stopped because something not on the map blocks
                        its path. Add what its lidar sees to the map, and plan a new path
                        for the current step from where the eBot stands.

        Args:
            data (PoseArray):    lidar returns around the eBot, in the map frame
        '''
        if self.grid is None or self.odom is None or self.state != 'drive' or not data.poses:
            return
        info = self.map_info
        res = info.resolution
        r = int(math.ceil(obstacle_grow_m / res))
        disc = np.hypot(*np.mgrid[-r:r + 1, -r:r + 1]) <= r
        for p in data.poses:
            col = int(math.floor((p.position.x - info.origin.position.x) / res))
            row = int(math.floor((p.position.y - info.origin.position.y) / res))
            if r <= row < info.height - r and r <= col < info.width - r:
                self.occupied[row - r:row + r + 1, col - r:col + r + 1] |= disc
            self.extra_obstacles.append((round(p.position.x, 3), round(p.position.y, 3)))
        self.extra_obstacles = self.extra_obstacles[-400:]
        self.build_clearance()

        x, y, yaw = self.odom[0], self.odom[1], self.odom[2]
        here = (x, y, yaw)
        end = steps[self.step][1]
        points = self.plan_path(here, end)
        if points is None:
            self.get_logger().error('No way round the new obstacle; waiting for it to clear.')
            return
        self.replans += 1
        self.get_logger().warn(f'Re-planned step {self.step + 1} around an obstacle not on the map '
                               f'(re-plan {self.replans}).')
        self.publish_path(points, here, end)


    def handoffcb(self, data):
        '''The arm reports whether it has taken the ore package off the eBot.'''
        self.handoff_done = bool(data.data)


    def odomcb(self, data):
        '''
        Description:    Store the position, the heading and the velocity of the base.

        Args:
            data (Odometry):    Pose and velocity of the base
        '''
        p = data.pose.pose.position
        q = data.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = data.twist.twist
        self.odom = (p.x, p.y, yaw, math.hypot(t.linear.x, t.linear.y), abs(t.angular.z))


    ##################### PLANNING #######################

    def to_cell(self, x, y):
        '''Map point (x, y) -> (row, col) in the planning grid.'''
        _, ox, oy, res = self.grid
        return int(math.floor((y - oy) / res)), int(math.floor((x - ox) / res))


    def to_point(self, row, col):
        '''Planning cell (row, col) -> its centre (x, y) in map.'''
        _, ox, oy, res = self.grid
        return ox + (col + 0.5) * res, oy + (row + 0.5) * res


    def clearance_at(self, row, col):
        '''Clearance in metres of a cell; 0 outside the grid.'''
        clearance = self.grid[0]
        if 0 <= row < clearance.shape[0] and 0 <= col < clearance.shape[1]:
            return float(clearance[row, col])
        return 0.0


    def astar(self, start, goal, hard):
        '''
        Description:    A* over the planning grid, 8-connected. Cells closer than `hard` to a
                        rock are blocked, except near the fixed start and goal, where cells
                        down to endpoint_clearance are allowed.

        Args:
            start (tuple):    (row, col) start cell
            goal (tuple):     (row, col) goal cell
            hard (float):     clearance in metres the body centre must keep

        Returns:
            list of (row, col) from start to goal, or None if there is no way through
        '''
        clearance = self.grid[0]
        res = self.grid[3]
        h, w = clearance.shape
        near = endpoint_radius / res
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                 (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

        def heuristic(c):
            return math.hypot(c[0] - goal[0], c[1] - goal[1])

        def allowed(r, c, cl):
            if cl >= hard:
                return True
            if cl < endpoint_clearance:
                return False
            return (math.hypot(r - start[0], c - start[1]) < near or
                    math.hypot(r - goal[0], c - goal[1]) < near)

        open_set = [(heuristic(start), 0.0, start)]
        came_from = {start: None}
        g_cost = {start: 0.0}
        while open_set:
            _, g, cell = heapq.heappop(open_set)
            if cell == goal:
                route = []
                while cell is not None:
                    route.append(cell)
                    cell = came_from[cell]
                return route[::-1]
            if g > g_cost[cell]:
                continue
            for dr, dc, step in moves:
                nr, nc = cell[0] + dr, cell[1] + dc
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                cl = clearance[nr, nc]
                if (nr, nc) != goal and not allowed(nr, nc, cl):
                    continue
                ng = g + step
                if ng < g_cost.get((nr, nc), math.inf):
                    g_cost[(nr, nc)] = ng
                    came_from[(nr, nc)] = cell
                    heapq.heappush(open_set, (ng + heuristic((nr, nc)), ng, (nr, nc)))
        return None


    def segment_clearance(self, p, q):
        '''Least clearance in metres along segment p-q, sampled on the full-resolution map.'''
        fine, res = self.fine
        _, ox, oy, _ = self.grid
        n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / res) + 1)
        t = np.linspace(0.0, 1.0, n)
        cols = np.floor((p[0] + t * (q[0] - p[0]) - ox) / res).astype(int)
        rows = np.floor((p[1] + t * (q[1] - p[1]) - oy) / res).astype(int)
        inside = (rows >= 0) & (rows < fine.shape[0]) & (cols >= 0) & (cols < fine.shape[1])
        if not np.all(inside):
            return 0.0
        return float(fine[rows, cols].min())


    def segment_clear(self, p, q, min_clear):
        '''True if every point on segment p-q has at least min_clear of clearance.'''
        return self.segment_clearance(p, q) >= min_clear


    def tighten(self, points, need):
        '''
        Description:    Pull each corner of a shortcut path towards the straight line
                        between its neighbours, as far as the clearance allows, so the path
                        wraps each rock at exactly the clearance asked for instead of at the
                        coarse grid's cell centres.

        Args:
            points (list):    [(x, y), ...] with fixed first and last points
            need (function):  need(p, q) -> clearance the leg p-q must keep

        Returns:
            list:             the same corners, moved, and any that became redundant removed
        '''
        pts = [np.array(p, dtype=float) for p in points]
        for _ in range(60):
            moved = False
            for i in range(1, len(pts) - 1):
                a, b, c = pts[i - 1], pts[i], pts[i + 1]
                if self.segment_clear(a, c, need(a, c)):
                    pts.pop(i)                          # the corner is no longer needed
                    moved = True
                    break
                # Try a step towards the chord midpoint, halving it until a clear one is found.
                d = (a + c) / 2.0 - b
                step = 1.0
                while step > 0.01:
                    nb = b + d * step * 0.5
                    if self.segment_clear(a, nb, need(a, nb)) and self.segment_clear(nb, c, need(nb, c)):
                        if np.linalg.norm(nb - b) > 1e-4:
                            pts[i] = nb
                            moved = True
                        break
                    step *= 0.5
            if not moved:
                break
        return [tuple(p) for p in pts]


    def leg_need(self, first, last, hard):
        '''
        Description:    The clearance rule for straight legs: `hard`, except that a leg
                        touching either fixed end may keep the lower clearance found there.

        Args:
            first (tuple):    (x, y) where the path starts
            last (tuple):     (x, y) where the path ends
            hard (float):     clearance asked for everywhere else

        Returns:
            function:         need(p, q) -> clearance the leg p-q must keep
        '''
        ends = []
        for end in (first, last):
            p = np.array(end, dtype=float)
            ends.append((p, max(endpoint_clearance, self.segment_clearance(p, p) - 0.01)))

        def need(p, q):
            lo = hard
            for end, end_clear in ends:
                if (math.hypot(p[0] - end[0], p[1] - end[1]) < endpoint_radius or
                        math.hypot(q[0] - end[0], q[1] - end[1]) < endpoint_radius):
                    lo = min(lo, end_clear)
            return lo
        return need


    def shortcut(self, points, hard):
        '''
        Description:    Straighten an A* route: from each point, jump to the furthest later
                        point the straight line reaches while keeping `hard` clearance. Near
                        the fixed ends the clearance the route already had is enough.

        Args:
            points (list):    [(x, y), ...] dense route
            hard (float):     clearance the straight lines must keep

        Returns:
            list:             [(x, y), ...] with the same ends and fewer, straighter legs
        '''
        if len(points) <= 2:
            return points
        need = self.leg_need(points[0], points[-1], hard)
        out = [points[0]]
        i = 0
        while i < len(points) - 1:
            j = len(points) - 1
            while j > i + 1 and not self.segment_clear(points[i], points[j], need(points[i], points[j])):
                j -= 1
            out.append(points[j])
            i = j
        return out


    def path_length(self, points):
        '''Length in metres of the polyline `points`.'''
        return sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(points[:-1], points[1:]))


    def plan_path(self, start, end):
        '''
        Description:    Plans a path from 'start' to 'end' that stays clear of every rock.
                        A straight line is used when it is clear; otherwise A* on the
                        clearance map, then shortcutting and tightening. Each clearance
                        level is tried, safest first, and the first path no longer than
                        the benchmark's (estimated at body_radius) is used.

        Args:
            start (tuple):    (x, y, yaw) where the step begins, in the map frame
            end (tuple):      (x, y, yaw) where the step ends, in the map frame

        Returns:
            list:             [(x, y), ...] from 'start' to 'end', or None if no path was found
        '''
        a = (start[0], start[1])
        b = (end[0], end[1])
        start_cell = self.to_cell(*a)
        goal_cell = self.to_cell(*b)

        def plan_at(hard):
            if self.segment_clear(a, b, hard):
                return [a, b]
            cells = self.astar(start_cell, goal_cell, hard)
            if cells is None:
                return None
            # Swap the end cells' centres for the exact start and end positions.
            points = [a] + [self.to_point(r, c) for r, c in cells[1:-1]] + [b]
            points = self.shortcut(points, hard)
            return self.tighten(points, self.leg_need(a, b, hard))

        # The benchmark's own path, as near as this planner gets to it.
        reference = plan_at(body_radius)
        ref_len = self.path_length(reference) if reference else math.inf
        self.get_logger().info(f'Benchmark estimate: {ref_len:.3f} m.')

        candidates = []                                 # (clearance, points), safest first
        for hard in clearance_levels:
            points = plan_at(hard)
            if points is None:
                self.get_logger().info(f'No path at {hard:.2f} m clearance.')
                continue
            length = self.path_length(points)
            self.get_logger().info(f'Path at {hard:.2f} m clearance: {len(points)} points, {length:.3f} m.')
            candidates.append((hard, points))
            if length <= ref_len + benchmark_slack:
                self.get_logger().info(f'Chose the {hard:.2f} m path: no longer than the benchmark.')
                return points
        if not candidates:
            return None
        # Nothing beat the benchmark: take the shortest found.
        hard, points = min(candidates, key=lambda c: self.path_length(c[1]))
        self.get_logger().info(f'Chose the {hard:.2f} m path, the shortest found.')
        return points


    def publish_path(self, points, start, end):
        '''
        Description:    Publishes 'points' on /ebot_path as a nav_msgs/Path. The first and
                        last poses carry the step's start and end yaw; the poses in between
                        face along the path.

        Args:
            points (list):    [(x, y), ...] as returned by plan_path()
            start (tuple):    (x, y, yaw) where the step begins
            end (tuple):      (x, y, yaw) where the step ends
        '''
        path = Path()
        path.header.frame_id = map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(points):
            if i == 0:
                yaw = start[2]
            elif i == len(points) - 1:
                yaw = end[2]
            else:
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = path.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
            path.poses.append(pose)

        self.path_pub.publish(path)
        length = self.path_length(points)
        if len(self.path_lengths) > self.step:          # a re-plan replaces this step's path
            self.path_lengths[self.step] = length
        else:
            self.path_lengths.append(length)
        self.get_logger().info(f'Step {self.step + 1}: published {len(points)} poses, '
                               f'{self.path_lengths[-1]:.2f} m.')


    def request_ore_package(self):
        '''
        Description:    Calls /spawn_ore_package once the eBot stands still on the ore drop
                        pose, without blocking the node. spawn_done() handles the reply.
        '''
        if self.spawn_pending:
            return
        if not self.spawn_client.service_is_ready():
            self.get_logger().info('Waiting for /spawn_ore_package ...', throttle_duration_sec=2.0)
            return
        self.spawn_pending = True
        future = self.spawn_client.call_async(Trigger.Request())
        future.add_done_callback(self.spawn_done)


    def spawn_done(self, future):
        '''
        Description:    Reply from /spawn_ore_package. On failure the timer asks again.

        Args:
            future (Future):    the completed call
        '''
        self.spawn_pending = False
        try:
            reply = future.result()
        except Exception as e:
            self.get_logger().error(f'/spawn_ore_package failed: {e}')
            return
        self.get_logger().info(f'/spawn_ore_package: success={reply.success}, message="{reply.message}"')
        if reply.success:
            self.spawned_at = self.get_clock().now()


    def at_pose(self, pose, check_yaw=True):
        '''True when the base is within the arrival tolerance of `pose`.'''
        x, y, yaw, _, _ = self.odom
        if math.hypot(pose[0] - x, pose[1] - y) > arrive_dist:
            return False
        return not check_yaw or abs(wrap_angle(pose[2] - yaw)) <= arrive_yaw


    def is_still(self):
        '''True when the base has no linear or angular velocity to speak of.'''
        return self.odom[3] < still_linear and self.odom[4] < still_angular


    def process_planning(self):
        '''
        Description:    Timer function: plan and publish each step while the eBot stands at
                        its start, wait until the follower has settled on its end, request
                        the package after step 1, and stop after step 3.
        '''
        try:
            self.planning_cycle()
        except Exception as e:      # an uncaught exception would kill the node
            self.get_logger().error(f'Planning error: {e}')
        msg = String()
        msg.data = json.dumps({'step': min(self.step + 1, len(steps)), 'steps': len(steps), 'state': self.state,
                               'path_lengths': [round(d, 3) for d in self.path_lengths],
                               'package': self.spawned_at is not None, 'replans': self.replans, 'handoff': self.handoff,
                               'obstacles': self.extra_obstacles[-200:]})
        self.status_pub.publish(msg)


    def planning_cycle(self):
        '''One planning cycle. See process_planning.'''
        if self.grid is None or self.odom is None or self.state == 'done':
            return
        now = self.get_clock().now()
        start, end = steps[self.step]

        if self.state == 'plan':
            # Plan only while the eBot stands still where the step begins.
            if not (self.is_still() and self.at_pose(start, check_yaw=False)):
                self.get_logger().info('Waiting for the eBot to settle at the step start ...',
                                       throttle_duration_sec=2.0)
                return
            points = self.plan_path(start, end)
            if points is None:
                self.get_logger().error(f'Step {self.step + 1}: no path found.', throttle_duration_sec=2.0)
                return
            self.publish_path(points, start, end)
            self.state = 'drive'
            self.still_since = None
            return

        if self.state == 'drive':
            if not (self.at_pose(end) and self.is_still()):
                self.still_since = None
                return
            if self.still_since is None:
                self.still_since = now
                return
            if (now - self.still_since).nanoseconds * 1e-9 < settle_time:
                return
            self.get_logger().info(f'Step {self.step + 1} complete.')
            if self.step == 0:
                self.state = 'package'
            elif self.step == 1 and self.handoff:
                self.state = 'handoff'
                self.handoff_ready_pub.publish(Bool(data=True))
                self.get_logger().info('At the arm pose: waiting for the arm to take the ore package.')
            else:
                self.state = 'next'

        if self.state == 'handoff':
            if not self.handoff_done:
                return
            self.handoff_ready_pub.publish(Bool(data=False))
            self.get_logger().info('The arm has the package. Heading home.')
            self.state = 'next'

        if self.state == 'package':
            if self.spawned_at is None:
                self.request_ore_package()
                return
            if (now - self.spawned_at).nanoseconds * 1e-9 < package_settle_time:
                return
            self.state = 'next'

        if self.state == 'next':
            self.step += 1
            if self.step >= len(steps):
                self.state = 'done'
                self.get_logger().info('Lap complete. Path lengths: ' +
                                       ', '.join(f'{d:.2f} m' for d in self.path_lengths) +
                                       f' (total {sum(self.path_lengths):.2f} m).')
                return
            self.state = 'plan'


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_planner class to perform its task
    '''

    rclpy.init(args=sys.argv)                                           # initialisation

    ebot_planner_class = ebot_planner()                                 # creating a new object for class 'ebot_planner'

    try:
        rclpy.spin(ebot_planner_class)                                  # spining on the object to make it alive in ROS 2 DDS
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ebot_planner_class.destroy_node()                               # destroy node after spin ends
        if rclpy.ok():
            rclpy.shutdown()                                            # shutdown process


if __name__ == '__main__':

    main()

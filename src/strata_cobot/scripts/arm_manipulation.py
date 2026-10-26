#!/usr/bin/env python3


'''
Strata Cobot - robotics course project, Amrita Vishwa Vidyapeetham.

Arm manipulation: picks each ore with the magnet and drops it in the container for its mineral.
'''

# Authors:          C Saihardhik Reddy, R Gagan Chowdary, K Charan Saatvikh Reddy
# Filename:         arm_manipulation.py
# Functions:
#			        origin_matrix, load_stl_points, main
#                   ArmModel: find_joint, link_poses, axis_rotation, tcp, jacobian,
#                             sigma_min, clearance, link_heights
#                   MotionPlanner: solve_ik, solve_from, converge, valid, path_ok, plan_to
#                   ore_sorter: descriptioncb, tcpposecb, jointstatecb, forcecb, armstatuscb,
#                               switch_controller, set_magnet, unlock_arm,
#                               publish_joint_velocity, servo_joints, servo_cartesian,
#                               find_ores, refresh_target, go, elapsed,
#                               process_sorting, step
# Nodes:		    ore_sorter_node
#			        Publishing Topics  - [ /delta_joint_cmds, /ore_sorter/status ]
#                   Subscribing Topics - [ /robot_description, /tcp_pose_raw, /joint_states,
#                                          /arm_status, /ur7e/end_effector/force_magnitude,
#                                          /tf, /tf_static ]
#                   Service Clients    - [ /magnet, /ur_arm_controller/switch_controller,
#                                          /ur_arm_controller/unlock_protective_stop ]


################### IMPORT MODULES #######################

import sys
import json
import math
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32, String
from std_srvs.srv import SetBool, Trigger
import tf2_ros


##################### TASK CONSTANTS #######################

servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'
base_frame = 'base_link'
tcp_frame = 'tcp'

joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# Container for each kind on the cargo rover. Seen from Gazebo's opening view (looking
# along +x): vanadinite nearest, malachite in the middle, azurite farthest. Their insides,
# from the rover's collision mesh in base_link, span y 0.489 to 0.889 and x:
#   vanadinite -0.755 to -0.499, malachite -0.489 to -0.182, azurite -0.172 to 0.115.
# Each ore is released over the point of its container nearest the arm, 8 cm in from the
# walls: the far corner of the vanadinite container is out of reach with the tool vertical.
containers = {
    'vanadinite_ore': (-0.580, 0.570),
    'malachite_ore':  (-0.336, 0.570),
    'azurite_ore':    (-0.029, 0.570),
}
ore_pattern = re.compile(r'^(azurite|malachite|vanadinite)_ore_\d+$')

# Fixed equipment around the arm, as axis-aligned boxes (min corner, max corner) in
# base_link: the conveyor, the ore crusher's body, its hopper walls, and the cargo rover
# with the container walls (whose rims are at the rover's top, z = -0.03).
obstacles = [
    ((-1.153, -0.933, -0.532), (0.155, -0.474, -0.142)),     # conveyor
    ((-1.223, -0.456, -0.488), (-0.635, 0.436, 0.043)),      # crusher body
    ((-1.234, 0.187, 0.037), (-0.648, 0.300, 0.207)),        # hopper wall, +y
    ((-1.251, -0.308, 0.037), (-0.665, -0.196, 0.207)),      # hopper wall, -y
    ((-1.236, -0.295, 0.037), (-1.123, 0.291, 0.207)),       # hopper wall, -x
    ((-0.763, -0.310, 0.037), (-0.650, 0.276, 0.207)),       # hopper wall, +x
    ((-1.061, 0.414, -0.564), (0.135, 0.964, -0.029)),       # cargo rover and containers
]
moving_links = ['upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link',
                'wrist_3_link', 'end_effector']

# What the servo accepts. A command above one of these is dropped WHOLE, not clamped.
joint_speed_rps = 0.30          # fastest joint, kept under the 0.60 rad/s cap
joint_gain = 1.5                # rad/s per rad of joint error
joint_settle_rad = 0.003        # joint error at which a target counts as reached
via_settle_rad = 0.03           # looser for detour points

# Planning margins.
limit_margin_rad = 0.3          # targets stay this far inside every joint limit
min_sigma = 0.04                # smallest Jacobian singular value allowed at a target
target_clearance_m = 0.03       # arm-to-equipment distance required at every target
path_clearance_m = 0.025        # ... and anywhere along a move
vertical_cos = math.cos(math.radians(1.0))     # tool axis within 1 degree of straight down
via_heights_m = (0.30, 0.40, 0.50)
# Tcp positions over open floor in front of the arm, used when no simple route is clear.
staging_points = [(0.30, -0.30, 0.35), (0.35, 0.0, 0.40), (0.30, 0.30, 0.35)]

# The ore: a 101.6 mm square block, 76.2 mm tall, held by its top face on the tcp.
ore_size = 0.1016
ore_height = 0.0762
hover_m = 0.10                  # above an ore's top face before the descent
carry_z = 0.16                  # tcp height for carrying (ore bottom 8 cm above the rover)
drop_z = 0.12                   # tcp height over a container when the ore is released

# The descent onto an ore, and the force at the tool (tool weight already removed).
descent_fast = 0.04             # m/s down to 3 cm above the face
descent_slow = 0.012            # m/s for the last 3 cm
press_m = 0.004                 # how far below the face the tcp is driven
contact_force_n = 2.5           # pressing this hard means the face is reached
held_force_n = 0.8              # an ore hanging on the tool reads about 1.47 N
lift_speed = 0.05
cart_gain = 2.0                 # 1/s, position error to tcp velocity
tilt_gain = 2.0                 # 1/s, tool tilt to angular velocity
cart_tol_m = 0.002
max_tries = 3


##################### HELPERS #######################

def origin_matrix(element):
    '''
    Purpose:
    ---
    4x4 transform of a URDF <origin> element (fixed-axis roll, pitch, yaw)

    Input Arguments:
    ---
    `element` :  [ Element or None ]    the <origin> tag

    Returns:
    ---
    `m` :  [ np.ndarray ]   4x4 transform; identity if the tag is absent
    '''
    m = np.eye(4)
    if element is None:
        return m
    xyz = [float(v) for v in element.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in element.get('rpy', '0 0 0').split()]
    m[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    m[:3, 3] = xyz
    return m


def load_stl_points(path):
    '''
    Purpose:
    ---
    Vertices of a binary STL file, de-duplicated

    Input Arguments:
    ---
    `path` :  [ str ]   file path

    Returns:
    ---
    `points` :  [ np.ndarray ]  Nx3
    '''
    with open(path, 'rb') as f:
        data = f.read()
    count = struct.unpack_from('<I', data, 80)[0]
    rec = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
    tris = np.frombuffer(data, dtype=rec, count=count, offset=84)
    return np.unique(tris['v'].reshape(-1, 3).astype(float).round(3), axis=0)


##################### ARM MODEL #######################

class ArmModel:
    '''
    ___CLASS___

    Description:    Kinematics and collision geometry of the arm, read from the same
                    robot_description the simulation runs, so the model matches the arm.
                    A held ore can be added to the geometry, hanging below the tcp.
    '''

    def __init__(self, urdf):
        '''
        Description:    Parse the URDF: the joint tree, and a point cloud of each moving
                        link's collision geometry in that link's frame.

        Args:
            urdf (str):    robot_description XML
        '''
        root = ET.fromstring(urdf)
        self.children = {}
        for j in root.findall('joint'):
            self.children.setdefault(j.find('parent').get('link'), []).append(j)

        # (parent, child, origin 4x4, axis, index into joint_names or None if not driven)
        self.tree = []
        stack = [base_frame]
        while stack:
            parent = stack.pop()
            for j in self.children.get(parent, []):
                name = j.get('name')
                axis = j.find('axis')
                axis = np.array([float(v) for v in axis.get('xyz').split()]) if axis is not None else None
                idx = joint_names.index(name) if name in joint_names else None
                child = j.find('child').get('link')
                self.tree.append((parent, child, origin_matrix(j.find('origin')), axis, idx))
                stack.append(child)

        self.geometry = {}
        for link in root.findall('link'):
            if link.get('name') not in moving_links:
                continue
            clouds = []
            for col in link.findall('collision'):
                origin = origin_matrix(col.find('origin'))
                geom = col.find('geometry')
                if geom.find('mesh') is not None:
                    path = geom.find('mesh').get('filename').replace('file://', '')
                    if not path.endswith('.stl'):
                        continue
                    pts = load_stl_points(path)
                elif geom.find('cylinder') is not None:
                    r = float(geom.find('cylinder').get('radius'))
                    h = float(geom.find('cylinder').get('length'))
                    pts = np.array([[r * math.cos(a), r * math.sin(a), z]
                                    for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)
                                    for z in np.linspace(-h / 2, h / 2, 3)])
                else:
                    continue
                clouds.append((origin[:3, :3] @ pts.T).T + origin[:3, 3])
            if clouds:
                pts = np.vstack(clouds)
                _, keep = np.unique(np.floor(pts / 0.015).astype(int), axis=0, return_index=True)
                self.geometry[link.get('name')] = pts[keep]

        # A held ore in the tcp frame: the tcp's z axis points out of the tool, so the ore
        # hangs along +z. Its corners and edge midpoints are enough against 2.5 cm margins.
        h = ore_size / 2
        self.payload = np.array([[x, y, z] for x in (-h, 0.0, h) for y in (-h, 0.0, h)
                                 for z in (0.0, ore_height / 2, ore_height)])
        self.carrying = False

        self.box_lo = np.array([b[0] for b in obstacles])
        self.box_hi = np.array([b[1] for b in obstacles])
        lower = [float(self.find_joint(n).find('limit').get('lower')) for n in joint_names]
        upper = [float(self.find_joint(n).find('limit').get('upper')) for n in joint_names]
        self.lower = np.array(lower) + limit_margin_rad
        self.upper = np.array(upper) - limit_margin_rad


    def find_joint(self, name):
        '''The <joint> element called `name`.'''
        for joints in self.children.values():
            for j in joints:
                if j.get('name') == name:
                    return j
        raise KeyError(name)


    def link_poses(self, q):
        '''
        Description:    Pose in base_link of every link below base_link, for joint angles q.

        Returns:
            dict link name -> 4x4 transform
        '''
        poses = {base_frame: np.eye(4)}
        for parent, child, origin, axis, idx in self.tree:
            m = poses[parent] @ origin
            if idx is not None:
                m = m @ self.axis_rotation(axis, q[idx])
            poses[child] = m
        return poses


    @staticmethod
    def axis_rotation(axis, angle):
        '''4x4 rotation by `angle` about the unit vector `axis` (Rodrigues).'''
        x, y, z = axis
        c, s = math.cos(angle), math.sin(angle)
        t = 1.0 - c
        return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
                         [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
                         [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
                         [0.0, 0.0, 0.0, 1.0]])


    def tcp(self, q):
        '''Pose of the tcp in base_link.'''
        return self.link_poses(q)[tcp_frame]


    def jacobian(self, q, eps=1e-6):
        '''Tcp Jacobian (linear over angular rows), by finite differences.'''
        m0 = self.tcp(q)
        jac = np.zeros((6, 6))
        for i in range(6):
            dq = np.array(q, dtype=float)
            dq[i] += eps
            m = self.tcp(dq)
            jac[:3, i] = (m[:3, 3] - m0[:3, 3]) / eps
            jac[3:, i] = Rotation.from_matrix(m[:3, :3] @ m0[:3, :3].T).as_rotvec() / eps
        return jac


    def sigma_min(self, q):
        '''Smallest singular value of the Jacobian: how far q is from a singularity.'''
        return float(np.linalg.svd(self.jacobian(q), compute_uv=False).min())


    def clearance(self, q, poses=None):
        '''Distance in metres from the arm (and a held ore) to the nearest obstacle box.'''
        poses = poses or self.link_poses(q)
        clouds = [(poses[name], pts) for name, pts in self.geometry.items()]
        if self.carrying:
            clouds.append((poses[tcp_frame], self.payload))
        best = math.inf
        for m, pts in clouds:
            p = (m[:3, :3] @ pts.T).T + m[:3, 3]
            gap = np.maximum(np.maximum(self.box_lo[None] - p[:, None], p[:, None] - self.box_hi[None]), 0.0)
            best = min(best, float(np.sqrt((gap ** 2).sum(axis=2)).min()))
        return best


    def link_heights(self, q, poses=None):
        '''Height of the elbow and wrist frames in base_link.'''
        poses = poses or self.link_poses(q)
        return np.array([poses[n][2, 3] for n in ('forearm_link', 'wrist_1_link',
                                                   'wrist_2_link', 'wrist_3_link')])


##################### PLANNER #######################

class MotionPlanner:
    '''
    ___CLASS___

    Description:    Joint configurations for tool positions with the tool pointing straight
                    down, and routes between them that keep the arm (and any held ore)
                    clear of the equipment and of singularities.
    '''

    def __init__(self, model, logger):
        self.model = model
        self.log = logger
        self.library = []               # [(tcp position, q)] of every configuration solved


    def solve_ik(self, position, seed):
        '''
        Description:    A configuration with the tcp on `position` and the tool pointing
                        straight down, clear of the equipment, joint limits and
                        singularities. Seeded from `seed` first, then from the solved
                        configurations nearest `position`, which is what lets the arm find
                        its way back to the conveyor from over the cargo rover.

        Returns:
            np.ndarray, or None if none is found
        '''
        target = np.array(position)
        nearest = sorted(self.library, key=lambda e: float(np.linalg.norm(e[0] - target)))[:3]
        for s in [np.array(seed, dtype=float)] + [q for _, q in nearest]:
            q = self.solve_from(target, s)
            if q is not None:
                self.library.append((target, q))
                return q
        return None


    def solve_from(self, target, seed, tries=6):
        '''
        Description:    solve_ik from one seed: the seed itself, then a few random
                        restarts around it. Each start is pulled onto the target by damped
                        least squares, and the result kept only if it passes every check.

        Returns:
            np.ndarray, or None if none is found
        '''
        rng = np.random.default_rng(6072)
        for trial in range(tries):
            start = seed if trial == 0 else seed + rng.normal(0.0, 0.5, 6)
            q = self.converge(target, start)
            if q is not None and self.valid(q):
                return q
        return None


    def converge(self, target, q, iters=60):
        '''
        Description:    Damped least-squares IK for five constraints: the tcp position, and
                        the tool axis straight down. The rotation about the vertical is
                        free, since the magnet face is round.

        Returns:
            np.ndarray once converged, or None
        '''
        q = np.array(q, dtype=float)
        down = np.array([0.0, 0.0, -1.0])
        for _ in range(iters):
            m = self.model.tcp(q)
            e_pos = target - m[:3, 3]
            e_rot = np.cross(m[:3, 2], down)            # rotation taking the tool axis to down
            if np.linalg.norm(e_pos) < 2e-5 and np.linalg.norm(e_rot) < 2e-4:
                return q
            jac = self.model.jacobian(q)[:5]            # x, y, z, and rotation about x and y
            err = np.concatenate((e_pos, e_rot[:2]))
            lam = 0.02
            dq = jac.T @ np.linalg.solve(jac @ jac.T + lam * lam * np.eye(5), err)
            n = float(np.linalg.norm(dq))
            if n > 0.3:
                dq *= 0.3 / n
            q = q + dq
        return None


    def valid(self, q):
        '''True if q is inside the joint limits, off the elbow and wrist singular branches,
        with the tool within a degree of vertical, and clear of the equipment.'''
        model = self.model
        if np.any(q < model.lower) or np.any(q > model.upper):
            return False
        if abs(q[2]) < 0.3 or abs(q[4]) < 0.3:
            return False
        poses = model.link_poses(q)
        if -poses[tcp_frame][2, 2] < vertical_cos:
            return False
        if np.any(model.link_heights(q, poses)[:3] < 0.0):
            return False
        if model.sigma_min(q) < min_sigma:
            return False
        return model.clearance(q, poses) >= target_clearance_m


    def path_ok(self, qa, qb):
        '''
        Description:    The joint servo sweeps the straight line from qa to qb in joint
                        space. True if that sweep keeps clear of the equipment and of
                        singularities (the first few percent excepted).
        '''
        steps = max(10, int(np.max(np.abs(qb - qa)) / 0.03))
        for s in np.linspace(0.0, 1.0, steps + 1)[1:]:
            q = qa + (qb - qa) * s
            if self.model.clearance(q) < path_clearance_m:
                return False
            if s > 0.1 and self.model.sigma_min(q) < min_sigma * 0.6:
                return False
        return True


    def plan_to(self, q_start, p_start, goal):
        '''
        Description:    A route from q_start (tcp at p_start) to the tool straight down at
                        `goal`: direct if the joint-space sweep is clear, otherwise up,
                        across and down through a via height.

        Returns:
            list of (q, is_final), or None if no route is found
        '''
        q_goal = self.solve_ik(goal, q_start)
        if q_goal is not None and self.path_ok(q_start, q_goal):
            return [(q_goal, True)]
        for h in via_heights_m:
            up = (p_start[0], p_start[1], max(h, p_start[2]))
            over = (goal[0], goal[1], max(h, goal[2]))
            q_up = self.solve_ik(up, q_start)
            q_over = self.solve_ik(over, q_up) if q_up is not None else None
            q_down = self.solve_ik(goal, q_over) if q_over is not None else None
            if q_down is None:
                continue
            if self.path_ok(q_start, q_up) and self.path_ok(q_up, q_over) and self.path_ok(q_over, q_down):
                return [(q_up, False), (q_over, False), (q_down, True)]
        # Through a staging point high over open floor, then down to the goal.
        for stage in staging_points:
            q_s = self.solve_ik(stage, q_start)
            if q_s is None or not self.path_ok(q_start, q_s):
                continue
            over = (goal[0], goal[1], max(stage[2], goal[2]))
            q_over = self.solve_ik(over, q_s)
            q_down = self.solve_ik(goal, q_over) if q_over is not None else None
            if q_down is not None and self.path_ok(q_s, q_over) and self.path_ok(q_over, q_down):
                return [(q_s, False), (q_over, False), (q_down, True)]
        if q_goal is not None:
            self.log.warn('No checked route; moving directly.')
            return [(q_goal, True)]
        return None


##################### CLASS DEFINITION #######################

class ore_sorter(Node):
    '''
    ___CLASS___

    Description:    Picks each ore off the conveyor with the magnet and drops it in the
                    container for its kind on the cargo rover. Large moves run in joint
                    space along collision-checked routes; the descent onto an ore and the
                    lift off it run as straight vertical tcp moves, resolved into joint
                    velocities through the arm's Jacobian.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_sorter
        '''
        super().__init__(
            'ore_sorter_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic PUBLISHERS ############

        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)
        self.status_pub = self.create_publisher(String, '/ore_sorter/status', 10)      # progress, as JSON, for the dashboard

        ############ Topic SUBSCRIPTIONS ############

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.desc_sub = self.create_subscription(String, '/robot_description', self.descriptioncb, latched_qos)
        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)
        self.force_sub = self.create_subscription(Float32, '/ur7e/end_effector/force_magnitude', self.forcecb, 10)

        ############ Service CLIENTS ############

        self.switch_cli = self.create_client(SwitchController, f'{servo_ns}/switch_controller')
        self.unlock_cli = self.create_client(Trigger, f'{servo_ns}/unlock_protective_stop')
        self.magnet_cli = self.create_client(SetBool, '/magnet')

        ############ Constructor VARIABLES/OBJECTS ############

        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.urdf = None
        self.tcp_pose = None            # tcp position in base_link, np.array(3)
        self.tcp_axis = None            # tool axis (tcp z) in base_link, np.array(3)
        self.joint_angles = None        # np.array(6), joint_names order
        self.joint_speed = None         # fastest joint, rad/s
        self.arm_status = None
        self.force = 0.0

        self.model = None
        self.planner = None
        self.ores = None                # [(name, kind, top-face position)] still to sort
        self.first_seen = None          # when ore transforms first appeared
        self.current = None             # the ore being handled
        self.tries = 0
        self.route = []                 # [(q, is_final)] being driven
        self.state = 'init'
        self.state_since = None
        self.resume_state = None        # the state a protective stop interrupted
        self.magnet_pending = False
        self.unlock_pending = False
        self.sorted = []
        self.cycles = 0

        self.joint_mode = self.switch_controller('joint_controller')

        control_rate = 0.02
        self.timer = self.create_timer(control_rate, self.process_sorting)

        self.get_logger().info('Arm manipulation node initialised.')


    ############ Callbacks ############

    def descriptioncb(self, data):
        '''Store the robot description, the source of the arm model.'''
        self.urdf = data.data


    def tcpposecb(self, data):
        '''Store where the tool is and which way it points.'''
        p = data.pose.position
        o = data.pose.orientation
        self.tcp_pose = np.array([p.x, p.y, p.z])
        self.tcp_axis = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()[:, 2]


    def jointstatecb(self, data):
        '''Store the joint angles and the fastest joint speed, matched by name.'''
        try:
            self.joint_angles = np.array([data.position[data.name.index(j)] for j in joint_names])
            if len(data.velocity) == len(data.name):
                self.joint_speed = max(abs(data.velocity[data.name.index(j)]) for j in joint_names)
        except (ValueError, IndexError):
            pass


    def forcecb(self, data):
        '''Store the force at the tool, in newtons, tool weight removed.'''
        self.force = float(data.data)


    def armstatuscb(self, data):
        '''Store the arm's state code, logging every change.'''
        if data.data != self.arm_status:
            if data.data == 0:
                self.get_logger().info('Arm status: 0 (healthy)')
            else:
                self.get_logger().warn(f'Arm status: {data.data}')
        self.arm_status = data.data


    ############ Services ############

    def switch_controller(self, controller):
        '''
        Description:    Make one of the arm's two command interfaces the active one. Called
                        from the constructor, before the node spins, so it may block.

        Args:
            controller  (str):      'twist_controller' or 'joint_controller'

        Returns:
            success     (bool):     Whether the controller was activated
        '''
        activate = joint_controller if controller == 'joint_controller' else twist_controller
        deactivate = twist_controller if activate == joint_controller else joint_controller
        if not self.switch_cli.wait_for_service(timeout_sec=20.0):
            self.get_logger().error('switch_controller service not available.')
            return False
        for attempt in range(5):
            req = SwitchController.Request()
            req.activate_controllers = [activate]
            req.deactivate_controllers = [deactivate]
            req.strictness = SwitchController.Request.STRICT
            future = self.switch_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if future.done() and future.result() is not None and future.result().ok:
                self.get_logger().info(f'Active input controller: {activate}')
                return True
            self.get_logger().warn(f'Switching to {activate} failed (attempt {attempt + 1}), retrying.')
        return False


    def set_magnet(self, on):
        '''
        Description:    Switch the magnet without blocking; magnet_pending clears when the
                        call has arrived.

        Args:
            on (bool):    True energises, False releases

        Returns:
            True if the call was sent
        '''
        if not self.magnet_cli.service_is_ready():
            return False
        self.magnet_pending = True
        req = SetBool.Request()
        req.data = bool(on)
        future = self.magnet_cli.call_async(req)

        def done(_):
            self.magnet_pending = False
        future.add_done_callback(done)
        self.get_logger().info(f'Magnet {"on" if on else "off"}.')
        return True


    def unlock_arm(self):
        '''Ask the servo to release a protective stop, without blocking.'''
        if self.unlock_pending or not self.unlock_cli.service_is_ready():
            return
        self.unlock_pending = True
        future = self.unlock_cli.call_async(Trigger.Request())

        def done(f):
            self.unlock_pending = False
            try:
                self.get_logger().warn(f'Unlock: {f.result().message}')
            except Exception:
                pass
        future.add_done_callback(done)


    ############ Motion ############

    def publish_joint_velocity(self, velocities):
        '''
        Description:    Send one JointJog, scaled as a whole to stay under the joint cap.

        Args:
            velocities (array):    rad/s for each joint, joint_names order
        '''
        v = np.asarray(velocities, dtype=float)
        if not np.all(np.isfinite(v)):
            v = np.zeros(6)
        peak = float(np.max(np.abs(v))) if len(v) else 0.0
        if peak > joint_speed_rps:
            v = v * (joint_speed_rps / peak)
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        msg.joint_names = joint_names
        msg.velocities = [float(x) for x in v]
        self.joint_pub.publish(msg)


    def servo_joints(self):
        '''
        Description:    Drive through self.route in joint space.

        Returns:
            True once the last configuration is reached and the arm has stopped
        '''
        if not self.route:
            self.publish_joint_velocity(np.zeros(6))
            return True
        target_q, final = self.route[0]
        q_err = target_q - self.joint_angles
        tol = joint_settle_rad if final else via_settle_rad
        if float(np.max(np.abs(q_err))) < tol:
            if not final:
                self.route.pop(0)
                return False
            self.publish_joint_velocity(np.zeros(6))
            if self.joint_speed is not None and self.joint_speed < 2e-3:
                self.route.pop(0)
                return True
            return False
        self.publish_joint_velocity(joint_gain * q_err)
        return False


    def servo_cartesian(self, goal, speed):
        '''
        Description:    Move the tcp in a straight line to `goal` at up to `speed`, holding
                        the tool straight down, by resolved-rate control on the model's
                        Jacobian with /tcp_pose_raw as feedback.

        Args:
            goal (np.ndarray):    tcp target in base_link
            speed (float):        m/s cap

        Returns:
            True once the tcp is within cart_tol_m of the goal (the arm is then stopped)
        '''
        err = goal - self.tcp_pose
        dist = float(np.linalg.norm(err))
        if dist < cart_tol_m:
            self.publish_joint_velocity(np.zeros(6))
            return True
        v = cart_gain * err
        n = float(np.linalg.norm(v))
        if n > speed:
            v *= speed / n
        # Tilt correction: rotate the tool axis towards straight down.
        w = tilt_gain * np.cross(self.tcp_axis, np.array([0.0, 0.0, -1.0]))
        jac = self.model.jacobian(self.joint_angles)
        qdot = np.linalg.pinv(jac, rcond=1e-3) @ np.concatenate((v, w))
        self.publish_joint_velocity(qdot)
        return False


    ############ Sorting ############

    def find_ores(self):
        '''
        Description:    The ores published by the perception node, read from tf2, with
                        the top-face position of each.

        Returns:
            list of (name, kind, position)
        '''
        frames = re.findall(r'^(\S+):', self.tf_buffer.all_frames_as_yaml(), re.MULTILINE)
        names = {f for f in frames if ore_pattern.match(f)}
        ores = []
        for name in sorted(names):
            try:
                t = self.tf_buffer.lookup_transform(base_frame, name, Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            p = t.transform.translation
            ores.append((name, name.rsplit('_', 1)[0], np.array([p.x, p.y, p.z])))
        return ores


    def refresh_target(self):
        '''Re-read the current ore's position from tf, in case it has been refined.'''
        try:
            t = self.tf_buffer.lookup_transform(base_frame, self.current[0], Time())
            p = t.transform.translation
            self.current = (self.current[0], self.current[1], np.array([p.x, p.y, p.z]))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass


    def go(self, state):
        '''Enter `state`, noting when.'''
        self.state = state
        self.state_since = self.get_clock().now()
        self.get_logger().info(f'-> {state}')


    def elapsed(self):
        '''Seconds since the current state began.'''
        return (self.get_clock().now() - self.state_since).nanoseconds * 1e-9


    def process_sorting(self):
        '''
        Description:    Timer function: one control cycle, guarded so an error stops the
                        arm instead of leaving the servo without a command.
        '''
        try:
            self.step()
        except Exception as e:
            self.get_logger().error(f'Control error: {e}')
            self.publish_joint_velocity(np.zeros(6))
        self.cycles += 1
        if self.cycles % 10 == 0:                                   # 5 Hz is plenty for people
            msg = String()
            msg.data = json.dumps({'state': self.state, 'current': self.current[0] if self.current else None,
                                   'sorted': self.sorted, 'remaining': [o[0] for o in (self.ores or [])],
                                   'tries': self.tries, 'force': round(self.force, 3), 'arm_status': self.arm_status})
            self.status_pub.publish(msg)


    def step(self):
        '''One control cycle. See process_sorting.'''
        if self.joint_angles is None or self.tcp_pose is None or self.arm_status is None \
                or self.urdf is None or not self.joint_mode:
            return

        # A protective stop: unlock, then back off upwards before carrying on.
        if 20 <= self.arm_status <= 23:
            self.publish_joint_velocity(np.zeros(6))
            self.unlock_arm()
            if self.state not in ('recover', 'done'):
                self.resume_state = self.state
                self.go('recover')
            return

        if self.state == 'init':
            self.model = ArmModel(self.urdf)
            self.planner = MotionPlanner(self.model, self.get_logger())
            self.go('wait_ores')
            return

        if self.state == 'recover':
            # Lift straight up for a moment, then re-plan the current step from here.
            if self.elapsed() < 1.5:
                self.publish_joint_velocity(np.linalg.pinv(self.model.jacobian(self.joint_angles), rcond=1e-3)
                                            @ np.array([0.0, 0.0, 0.03, 0.0, 0.0, 0.0]))
                return
            self.publish_joint_velocity(np.zeros(6))
            self.route = []
            back = {'descend': 'plan_pick', 'grip': 'lift', 'to_ore': 'plan_pick',
                    'align': 'plan_pick', 'to_bin': 'plan_drop'}.get(self.resume_state, self.resume_state)
            self.go(back)
            return

        if self.state == 'wait_ores':
            self.publish_joint_velocity(np.zeros(6))
            ores = self.find_ores()
            if not ores:
                return
            if self.first_seen is None:
                self.first_seen = self.get_clock().now()
            # Give the detector a moment to publish every ore before fixing the list.
            if (self.get_clock().now() - self.first_seen).nanoseconds * 1e-9 < 3.0:
                return
            self.ores = self.find_ores()
            for name, _, p in self.ores:
                self.get_logger().info(f'Ore {name} at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}).')
            self.go('next')
            return

        if self.state == 'next':
            if not self.ores:
                self.go('park')
                return
            # Nearest the current tool position first: shorter moves.
            self.ores.sort(key=lambda o: float(np.linalg.norm(o[2][:2] - self.tcp_pose[:2])))
            self.current = self.ores.pop(0)
            self.tries = 0
            self.go('plan_pick')
            return

        if self.state == 'plan_pick':
            self.publish_joint_velocity(np.zeros(6))
            self.refresh_target()
            p = self.current[2]
            hover = (p[0], p[1], p[2] + hover_m)
            self.model.carrying = False
            self.route = self.planner.plan_to(self.joint_angles.copy(), self.tcp_pose.copy(), hover) or []
            if not self.route:
                self.get_logger().error(f'No way to reach {self.current[0]}; skipping it.')
                self.go('next')
                return
            self.get_logger().info(f'Picking {self.current[0]}: {len(self.route)} moves.')
            self.go('to_ore')
            return

        if self.state == 'to_ore':
            if self.servo_joints():
                self.go('align')
            return

        if self.state == 'align':
            # Close the last few mm between the model and the arm, above the ore.
            p = self.current[2]
            if self.servo_cartesian(np.array([p[0], p[1], p[2] + hover_m]), 0.03):
                if self.set_magnet(True):
                    self.go('descend')
            return

        if self.state == 'descend':
            p = self.current[2]
            goal = np.array([p[0], p[1], p[2] - press_m])
            speed = descent_fast if self.tcp_pose[2] > p[2] + 0.03 else descent_slow
            if self.force > contact_force_n or self.servo_cartesian(goal, speed):
                self.publish_joint_velocity(np.zeros(6))
                self.get_logger().info(f'Contact: tcp z {self.tcp_pose[2]:.3f}, face {p[2]:.3f}, '
                                       f'force {self.force:.2f} N.')
                self.go('grip')
            return

        if self.state == 'grip':
            self.publish_joint_velocity(np.zeros(6))
            if self.elapsed() > 0.5:
                self.go('lift')
            return

        if self.state == 'lift':
            p = self.current[2]
            if self.servo_cartesian(np.array([p[0], p[1], max(p[2] + hover_m, carry_z)]), lift_speed):
                self.go('check')
            return

        if self.state == 'check':
            self.publish_joint_velocity(np.zeros(6))
            if self.elapsed() < 0.5:
                return
            if self.force > held_force_n:
                self.get_logger().info(f'{self.current[0]} held ({self.force:.2f} N).')
                self.go('plan_drop')
                return
            self.tries += 1
            self.get_logger().warn(f'{self.current[0]} not held ({self.force:.2f} N), try {self.tries}.')
            self.set_magnet(False)
            self.go('next' if self.tries >= max_tries else 'plan_pick')
            return

        if self.state == 'plan_drop':
            self.publish_joint_velocity(np.zeros(6))
            cx, cy = containers[self.current[1]]
            self.model.carrying = True
            self.route = self.planner.plan_to(self.joint_angles.copy(), self.tcp_pose.copy(),
                                              (cx, cy, drop_z)) or []
            if not self.route:
                self.get_logger().error('No way to the container; releasing here.')
                self.set_magnet(False)
                self.go('release')
                return
            self.get_logger().info(f'Carrying {self.current[0]} to its container: {len(self.route)} moves.')
            self.go('to_bin')
            return

        if self.state == 'to_bin':
            if self.servo_joints():
                if self.set_magnet(False):
                    self.go('release')
            return

        if self.state == 'release':
            self.publish_joint_velocity(np.zeros(6))
            if self.elapsed() < 1.0:
                return
            self.model.carrying = False
            self.sorted.append(self.current[0])
            self.get_logger().info(f'{self.current[0]} dropped ({self.force:.2f} N on the tool). '
                                   f'Sorted {len(self.sorted)}.')
            self.go('next')
            return

        if self.state == 'park':
            self.publish_joint_velocity(np.zeros(6))
            self.get_logger().info(f'All ores handled: {", ".join(self.sorted)}. Done.')
            self.go('done')
            return

        if self.state == 'done':
            self.publish_joint_velocity(np.zeros(6))


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ore_sorter class to perform its task
    '''
    rclpy.init(args=sys.argv)
    ore_sorter_class = ore_sorter()
    try:
        rclpy.spin(ore_sorter_class)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ore_sorter_class.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()

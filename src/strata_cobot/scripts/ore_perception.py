#!/usr/bin/env python3


'''
Strata Cobot - robotics course project, Amrita Vishwa Vidyapeetham.

Ore perception: finds each ore by its shape in the depth image, classifies its mineral by
a vote over the colours of its whole top face, and publishes its position as a tf2 frame.
Real ores are rarely one clean colour, so a face counts as a mineral when enough of it,
not all of it, shows that mineral's colour.
'''

# Authors:          C Saihardhik Reddy, R Gagan Chowdary, K Charan Saatvikh Reddy
# Filename:         ore_perception.py
# Functions:
#			        back_project, find_faces, colour_shares, classify_face, main
#                   ore_tf: depthimagecb, colorimagecb, caminfocb, camera_to_base,
#                           detect, name_ores, track_ores, publish_transforms,
#                           process_image, step
# Nodes:		    ore_tf_publisher
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw,
#                                          /camera/camera/aligned_depth_to_color/image_raw,
#                                          /camera/camera/color/camera_info, /tf, /tf_static ]


################### IMPORT MODULES #######################

import sys
from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
import tf2_ros


##################### CONSTANTS #######################

color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'
base_frame = 'base_link'

# Where to look: the conveyor's top surface in base_link, from the arena's collision mesh.
conveyor_lo = np.array([-1.16, -0.94])
conveyor_hi = np.array([0.16, -0.46])
belt_z = -0.142

# What an ore looks like to the depth camera: a 101.6 mm square block, 76.2 mm tall. Its top
# face is the only surface 66-86 mm above the belt, and that band leaves a gap between ores
# standing close together, where their sides would join up.
face_band = (0.066, 0.086)              # height above the belt, m
face_side_m = (0.080, 0.130)            # accepted length of each side of the fitted face
min_face_px = 80

# The colour of each mineral, as HSV ranges (OpenCV: H 0-180, S and V 0-255). Hue carries
# the mineral; saturation and value only have to rule out grey, black and white matrix.
# Orange is held to a brighter, more saturated range so that brown rock does not count.
mineral_colours = {
    'azurite_ore':    [((95, 80, 40), (130, 255, 255))],                              # blue
    'malachite_ore':  [((40, 80, 40), (90, 255, 255))],                               # green
    'vanadinite_ore': [((0, 140, 100), (20, 255, 255)), ((170, 140, 100), (180, 255, 255))],  # orange-red
}
# Shares are of the whole top face. A clean simulated ore shows its colour on about 46% of
# it (a coloured square inset in a black face), so a mineral needs a tenth of the face and
# twice the share of any other colour; otherwise the face is 'uncertain'.
min_share = 0.10
min_ratio = 2.0

naming_frames = 20                      # frames watched before the names are fixed
history_len = 30                        # frames averaged into each published position
same_ore_m = 0.05                       # detections closer than this are the same ore


##################### FUNCTION DEFINITION #######################

def back_project(depth, intrinsics, cam_to_base):
    '''
    Purpose:
    ---
    Every pixel of an aligned depth image as a point in base_link

    Input Arguments:
    ---
    `depth` :        [ np.ndarray ]   HxW depth in metres, NaN where unknown
    `intrinsics` :   [ tuple ]        (fx, fy, cx, cy)
    `cam_to_base` :  [ np.ndarray ]   4x4 transform, optical frame -> base_link

    Returns:
    ---
    `xyz` :  [ np.ndarray ]   HxWx3 points in base_link (NaN where the depth is)
    '''
    fx, fy, cx, cy = intrinsics
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    pts = np.stack(((u - cx) * depth / fx, (v - cy) * depth / fy, depth), axis=-1)
    return pts @ cam_to_base[:3, :3].T + cam_to_base[:3, 3]


def find_faces(xyz):
    '''
    Purpose:
    ---
    Find each ore's top face by shape alone: the regions of the conveyor that stand at an
    ore's height above the belt and are the size of an ore's face. Colour plays no part,
    so an ore is found however mixed its colour is.

    Input Arguments:
    ---
    `xyz` :  [ np.ndarray ]   HxWx3 points in base_link, from back_project()

    Returns:
    ---
    `faces` :  [ list ]
        (mask, centre) per face: a uint8 mask of its pixels, and the centre of the face in
        base_link, from a rectangle fitted to its outline
    '''
    x, y, z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
    with np.errstate(invalid='ignore'):
        band = ((x > conveyor_lo[0]) & (x < conveyor_hi[0]) & (y > conveyor_lo[1]) & (y < conveyor_hi[1])
                & (z > belt_z + face_band[0]) & (z < belt_z + face_band[1]))
    mask = cv2.morphologyEx(band.astype(np.uint8) * 255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    faces = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_face_px:
            continue
        face = labels == i
        # The centre of a rectangle fitted to the face, rather than the median of its
        # points: pixels on the edge blend in depth with what lies behind them and would
        # pull a median towards the camera by several millimetres.
        pts = np.column_stack((x[face], y[face])).astype(np.float32) * 1000.0
        (rx, ry), (rw, rh), _ = cv2.minAreaRect(pts)
        if not (face_side_m[0] * 1000 <= min(rw, rh) and max(rw, rh) <= face_side_m[1] * 1000):
            continue                                        # not the size of an ore face
        centre = np.array([rx / 1000.0, ry / 1000.0, float(np.nanmedian(z[face]))])
        faces.append((face.astype(np.uint8) * 255, centre))
    return faces


def colour_shares(hsv, mask):
    '''
    Purpose:
    ---
    The share of a region's pixels showing each mineral's colour

    Input Arguments:
    ---
    `hsv` :   [ np.ndarray ]   HxWx3 image in OpenCV HSV
    `mask` :  [ np.ndarray ]   uint8 mask of the region

    Returns:
    ---
    `shares` :  [ dict ]   mineral -> fraction of the region's pixels, 0 to 1
    '''
    total = max(1, cv2.countNonZero(mask))
    shares = {}
    for mineral, ranges in mineral_colours.items():
        hit = np.zeros(mask.shape, np.uint8)
        for lo, hi in ranges:
            hit |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        shares[mineral] = cv2.countNonZero(hit & mask) / total
    return shares


def classify_face(hsv, mask):
    '''
    Purpose:
    ---
    The mineral of a face, by a vote over all of its pixels: the colour with the largest
    share wins, provided it covers at least min_share of the face and has min_ratio times
    the share of the next colour. Veins, dust and dark matrix only lower the winner's share.

    Input Arguments:
    ---
    `hsv` :   [ np.ndarray ]   HxWx3 image in OpenCV HSV
    `mask` :  [ np.ndarray ]   uint8 mask of the face

    Returns:
    ---
    `mineral` :  [ str or None ]   None when no colour clearly wins ('uncertain')
    `shares` :   [ dict ]          mineral -> share of the face
    '''
    inner = cv2.erode(mask, np.ones((3, 3), np.uint8))     # edge pixels mix with the background
    if cv2.countNonZero(inner) >= min_face_px // 2:
        mask = inner
    shares = colour_shares(hsv, mask)
    ranked = sorted(shares.items(), key=lambda kv: kv[1], reverse=True)
    (best, s1), (_, s2) = ranked[0], ranked[1]
    if s1 < min_share or s1 < min_ratio * s2:
        return None, shares
    return best, shares


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Finds the ores on the conveyor by shape in the depth image, classifies
                    each by a vote over its face's colours, names them, and broadcasts one
                    tf2 transform per ore from base_link to the centre of its top face.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_tf
        '''
        super().__init__(
            'ore_tf_publisher',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.1
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(image_processing_rate, self.process_image)

        self.cv_image = None
        self.depth_image = None
        self.intrinsics = None          # (fx, fy, cx, cy)
        self.optical_frame = None
        self.cam_to_base = None         # 4x4, optical frame -> base_link (the camera is fixed)

        self.frames_seen = 0
        self.candidates = []            # [(mineral or None, position)] gathered before naming
        self.history = {}               # ore name -> recent face positions in base_link
        self.named = False

        self.get_logger().info('Ore perception node initialised.')


    def depthimagecb(self, data):
        '''
        Description:    Store the aligned depth image, in metres.

        Args:
            data (Image):    Depth image aligned to the colour image
        '''
        try:
            depth = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'Depth conversion error: {e}')
            return
        depth = depth.astype(np.float32)
        if data.encoding == '16UC1':
            depth /= 1000.0                                 # millimetres to metres
        depth[~np.isfinite(depth) | (depth <= 0.0)] = np.nan
        self.depth_image = depth


    def colorimagecb(self, data):
        '''
        Description:    Store the colour image.

        Args:
            data (Image):    Colour image from the camera
        '''
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Color conversion error: {e}')


    def caminfocb(self, data):
        '''
        Description:    Store the camera intrinsics and the frame they belong to.

        Args:
            data (CameraInfo):    Calibration of the colour camera
        '''
        k = data.k
        self.intrinsics = (k[0], k[4], k[2], k[5])
        self.optical_frame = data.header.frame_id


    def camera_to_base(self):
        '''
        Description:    The optical frame -> base_link transform, looked up once; the
                        camera is fixed to the arm's base.

        Returns:
            np.ndarray 4x4, or None until tf has it
        '''
        if self.cam_to_base is not None:
            return self.cam_to_base
        try:
            tf = self.tf_buffer.lookup_transform(base_frame, self.optical_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        m = np.eye(4)
        r = tf.transform.rotation
        m[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        tr = tf.transform.translation
        m[:3, 3] = [tr.x, tr.y, tr.z]
        self.cam_to_base = m
        return m


    def detect(self, cam_to_base):
        '''
        Description:    One frame: every ore face on the conveyor, with its mineral.

        Args:
            cam_to_base (np.ndarray):   4x4 transform, optical frame -> base_link

        Returns:
            list of (mineral or None, centre in base_link)
        '''
        xyz = back_project(self.depth_image, self.intrinsics, cam_to_base)
        hsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        found = []
        for mask, centre in find_faces(xyz):
            mineral, _ = classify_face(hsv, mask)
            found.append((mineral, centre))
        return found


    def name_ores(self):
        '''
        Description:    Fix the ore names from the detections gathered so far. Detections
                        closer than same_ore_m are one ore, whatever their colour; its
                        mineral is the majority vote of its frames. Each mineral is then
                        numbered from 1, nearest the arm base first.
        '''
        groups = []                                     # [[(mineral, position)]]
        for mineral, p in self.candidates:
            for g in groups:
                if np.linalg.norm(np.median([q for _, q in g], axis=0) - p) < same_ore_m:
                    g.append((mineral, p))
                    break
            else:
                groups.append([(mineral, p)])

        # A real ore is seen in most frames; a stray region in only a few.
        min_hits = max(3, naming_frames // 3)
        by_mineral = {}
        for g in groups:
            if len(g) < min_hits:
                continue
            votes = Counter(m for m, _ in g if m is not None)
            if not votes:
                self.get_logger().warn(f'Ore at {np.median([q for _, q in g], axis=0).round(3)} '
                                       'could not be classified; skipping it.')
                continue
            mineral, count = votes.most_common(1)[0]
            by_mineral.setdefault(mineral, []).append([q for _, q in g])
            self.get_logger().info(f'Ore classified as {mineral} in {count} of {len(g)} frames.')

        for mineral, ores in by_mineral.items():
            ores.sort(key=lambda g: float(np.linalg.norm(np.median(g, axis=0)[:2])))
            for i, g in enumerate(ores):
                name = f'{mineral}_{i + 1}'
                self.history[name] = deque(g[-history_len:], maxlen=history_len)
                x, y, z = np.median(g, axis=0)
                self.get_logger().info(f'{name}: ({x:.3f}, {y:.3f}, {z:.3f}) in {base_frame}')
        self.named = True


    def track_ores(self, found):
        '''
        Description:    Add each detection to the history of the named ore nearest to it, if
                        it is close enough and shows that ore's mineral. A face partly hidden
                        by the arm is left out rather than allowed to move the ore.

        Args:
            found (list):   [(mineral or None, position)] this frame
        '''
        for mineral, p in found:
            best, best_d = None, same_ore_m
            for name, hist in self.history.items():
                d = float(np.linalg.norm(np.median(np.array(hist), axis=0) - p))
                if d < best_d:
                    best, best_d = name, d
            if best is not None and mineral is not None and best.startswith(mineral):
                self.history[best].append(p)


    def publish_transforms(self):
        '''
        Description:    Broadcast every named ore at the median of its recent positions.
                        Called every cycle, so the transforms never lapse.
        '''
        stamp = self.get_clock().now().to_msg()
        for name, hist in self.history.items():
            x, y, z = np.median(np.array(hist), axis=0)
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = base_frame
            t.child_frame_id = name
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = float(z)
            t.transform.rotation.w = 1.0
            self.br.sendTransform(t)


    def process_image(self):
        '''
        Description:    Timer function: detect, name or track, and publish.
        '''
        try:
            self.step()
        except Exception as e:      # keep publishing what is known even if one frame fails
            self.get_logger().error(f'Processing error: {e}')
        self.publish_transforms()


    def step(self):
        '''One processing cycle. See process_image.'''
        if self.cv_image is None or self.depth_image is None or self.intrinsics is None:
            return
        cam_to_base = self.camera_to_base()
        if cam_to_base is None:
            return

        found = self.detect(cam_to_base)
        if self.named:
            self.track_ores(found)
            return
        self.candidates.extend(found)
        self.frames_seen += 1
        if self.frames_seen >= naming_frames:
            self.name_ores()


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ore_tf class to perform its task
    '''
    rclpy.init(args=sys.argv)
    ore_tf_class = ore_tf()
    try:
        rclpy.spin(ore_tf_class)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ore_tf_class.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':

    main()

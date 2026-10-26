#!/usr/bin/env python3


'''
Strata Cobot - robotics course project, Amrita Vishwa Vidyapeetham.

Ore perception: detects each ore's coloured face in the RGB-D camera and publishes its position as a tf2 frame.
'''

# Authors:          C Saihardhik Reddy, R Gagan Chowdary, K Charan Saatvikh Reddy
# Filename:         ore_perception.py
# Functions:
#			        detect_ores, on_conveyor, main
#                   ore_tf: depthimagecb, colorimagecb, caminfocb, camera_to_base,
#                           locate_face, name_ores, track_ores, publish_transforms,
#                           process_image, step
# Nodes:		    ore_tf_publisher
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw,
#                                          /camera/camera/aligned_depth_to_color/image_raw,
#                                          /camera/camera/color/camera_info, /tf, /tf_static ]


################### IMPORT MODULES #######################

import sys
from collections import deque

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


##################### TASK CONSTANTS #######################

color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'
base_frame = 'base_link'

# HSV bounds of each ore's coloured top face (OpenCV: H 0-180, S and V 0-255), from
# camera frames. The faces are far more saturated than the orange terrain around them.
color_ranges = {
    'azurite_ore':    ((98, 200, 100), (112, 255, 255)),     # blue
    'malachite_ore':  ((64, 200, 120), (84, 255, 255)),      # green
    'vanadinite_ore': ((5, 225, 180), (14, 255, 255)),       # orange
}
total_ores = 4

min_face_area_px = 60           # smaller blobs are noise, not an ore face
max_face_area_px = 4000
min_fill_ratio = 0.6            # face area / its rotated bounding box: faces are square

# The conveyor's top in base_link, from the arena's collision mesh, with room for the
# ore standing on it. Only faces in here are ores waiting to be picked: one on the tool
# or in a container is ignored, so its transform keeps the conveyor position.
conveyor_lo = np.array([-1.16, -0.94, -0.10])
conveyor_hi = np.array([0.16, -0.46, -0.03])

naming_frames = 20              # frames watched before the names are fixed
history_len = 30                # frames averaged into each published position
same_ore_m = 0.05               # detections closer than this are the same ore


##################### FUNCTION DEFINITION #######################

def detect_ores(image):
    '''
    Purpose:
    ---
    Find the coloured top face of every ore in a frame

    Input Arguments:
    ---
    `image` :  [ np.ndarray ]   BGR colour frame

    Returns:
    ---
    `detections` :  [ list ]
        (ore type, mask of the face) for each face found
    '''
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    detections = []
    for ore_type, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if not min_face_area_px <= area <= max_face_area_px:
                continue
            (_, _), (w, h), _ = cv2.minAreaRect(c)
            if w * h <= 0 or area / (w * h) < min_fill_ratio:
                continue
            face = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(face, [c], -1, 255, cv2.FILLED)
            detections.append((ore_type, face))
    return detections


def on_conveyor(p):
    '''
    Purpose:
    ---
    Whether a face centre in base_link lies on the conveyor

    Input Arguments:
    ---
    `p` :  [ np.ndarray ]   (x, y, z) in base_link

    Returns:
    ---
    `inside` :  [ bool ]
    '''
    return bool(np.all(p >= conveyor_lo) and np.all(p <= conveyor_hi))


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Detects the ores' coloured top faces in the camera image, locates
                    each face in 3D from the aligned depth image, names each ore by its
                    kind, and broadcasts one tf2 transform per ore from base_link to the
                    centre of its top face.
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
        self.candidates = []            # [(ore type, position)] gathered before naming
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


    def locate_face(self, face_mask, cam_to_base):
        '''
        Description:    Position of a top face's centre in base_link. Every pixel of the
                        face is back-projected with its own depth and moved into
                        base_link; the median of those points is the face centre, which is
                        robust to the noisy pixels along the face's edge.

        Args:
            face_mask (np.ndarray):     255 on the face's pixels
            cam_to_base (np.ndarray):   4x4 transform, optical frame -> base_link

        Returns:
            np.ndarray(3), or None if the face has no depth
        '''
        inner = cv2.erode(face_mask, np.ones((3, 3), np.uint8))
        if cv2.countNonZero(inner) < 10:
            inner = face_mask
        v, u = np.nonzero(inner)
        z = self.depth_image[v, u]
        ok = np.isfinite(z)
        if np.count_nonzero(ok) < 5:
            return None
        u, v, z = u[ok].astype(float), v[ok].astype(float), z[ok]
        fx, fy, cx, cy = self.intrinsics
        pts = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z, np.ones_like(z)))
        pts_base = (cam_to_base @ pts.T).T[:, :3]
        return np.median(pts_base, axis=0)


    def name_ores(self):
        '''
        Description:    Fix the ore names from the detections gathered so far. Detections of
                        one kind closer than same_ore_m are one ore; each kind is then
                        numbered from 1, nearest the arm base first.
        '''
        clusters = {}                                   # ore type -> [[positions]]
        for ore_type, p in self.candidates:
            groups = clusters.setdefault(ore_type, [])
            for g in groups:
                if np.linalg.norm(np.median(g, axis=0) - p) < same_ore_m:
                    g.append(p)
                    break
            else:
                groups.append([p])

        # A real ore is seen in most frames; a stray blob in only a few.
        min_hits = max(3, naming_frames // 3)
        for ore_type, groups in clusters.items():
            groups = [g for g in groups if len(g) >= min_hits]
            groups.sort(key=lambda g: float(np.linalg.norm(np.median(g, axis=0)[:2])))
            for i, g in enumerate(groups):
                name = f'{ore_type}_{i + 1}'
                self.history[name] = deque(g[-history_len:], maxlen=history_len)
                x, y, z = np.median(g, axis=0)
                self.get_logger().info(f'{name}: ({x:.3f}, {y:.3f}, {z:.3f}) in {base_frame}')
        self.named = True
        if len(self.history) != total_ores:
            self.get_logger().warn(f'{len(self.history)} ores named, expected {total_ores}.')


    def track_ores(self, found):
        '''
        Description:    Add each detection to the history of the named ore of its kind
                        nearest to it, if it is close enough to be that ore.

        Args:
            found (list):   [(ore type, position)] on the conveyor this frame
        '''
        for ore_type, p in found:
            best, best_d = None, same_ore_m
            for name, hist in self.history.items():
                if not name.startswith(ore_type):
                    continue
                d = float(np.linalg.norm(np.median(np.array(hist), axis=0) - p))
                if d < best_d:
                    best, best_d = name, d
            if best is not None:
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
        Description:    Timer function: detect, locate, name or track, and publish.
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

        found = []
        for ore_type, face in detect_ores(self.cv_image):
            p = self.locate_face(face, cam_to_base)
            if p is not None and on_conveyor(p):
                found.append((ore_type, p))

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

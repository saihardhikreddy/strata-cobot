#!/usr/bin/env python3
'''
Strata Cobot dashboard: one-click runs and a live view of both robots.

A small web app on http://localhost:8765. Its buttons launch the simulated world and the
robot nodes for either mission (ore sorting with the arm, or the mobile robot's lap), and
the page mirrors what the robots see and do, live:

  * ore sorting: the arm camera with every detected ore outlined and labelled, each ore's
    colour vote, the arm's current step, the magnet force and the sorted count;
  * mobile robot lap: the arena map with the planned paths, the robot's position and trail,
    and the lap's progress.

Run it with dashboard/run_dashboard.sh, which sources ROS 2 first. No extra packages are
needed beyond ROS 2 Jazzy (rclpy, cv_bridge) and OpenCV.
'''
import argparse
import collections
import json
import math
import os
import signal
import subprocess
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import cv2
import numpy as np
import socket

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, String

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCRIPTS = os.path.join(ROOT, 'src', 'strata_cobot', 'scripts')
STATIC = os.path.join(HERE, 'static')

# Each mission: the simulated world to launch, the nodes to start once it is ready, and the
# log line that means the mission is over.
MISSIONS = {
    'sorting': {'launch': 'task2a.launch.py', 'nodes': ['ore_perception.py', 'arm_manipulation.py'],
                'done': 'All ores handled'},
    'lap': {'launch': 'task2b.launch.py', 'nodes': ['path_planning.py', 'path_following.py'],
            'done': 'Lap complete'},
}
WORLD_READY = 'world ready'
MINERAL_BGR = {'azurite_ore': (230, 130, 40), 'malachite_ore': (70, 200, 60), 'vanadinite_ore': (30, 110, 245)}


##################### MISSION RUNNER #######################

class MissionRunner:
    '''Starts and stops the simulated world and the robot nodes, and keeps their log.'''

    def __init__(self, on_new_run):
        self.lock = threading.Lock()
        self.procs = []                     # [(label, Popen)]
        self.mode = None
        self.phase = 'idle'                 # idle, starting, running, done, stopping, error
        self.message = 'Press a button to start a mission.'
        self.started = None
        self.finished = None
        self.log = collections.deque(maxlen=400)
        self.on_new_run = on_new_run

    def snapshot(self):
        with self.lock:
            end = self.finished or time.time()
            return {'mode': self.mode, 'phase': self.phase, 'message': self.message,
                    'elapsed': round(end - self.started, 1) if self.started else 0.0,
                    'log': list(self.log)[-80:]}

    def note(self, text):
        self.log.append(time.strftime('%H:%M:%S ') + text)

    def start(self, mode, gazebo):
        with self.lock:
            if self.phase in ('starting', 'running', 'stopping'):
                return False, 'A mission is already running. Stop it first.'
            if subprocess.run(['pgrep', '-f', 'gz sim'], capture_output=True).returncode == 0:
                return False, 'Another simulation is already running on this computer. Close it first.'
            self.mode, self.phase = mode, 'starting'
            self.message = 'Launching the simulated world (about a minute)...'
            self.started, self.finished = time.time(), None
            self.log.clear()
        self.on_new_run(mode)
        self.note(f'Starting {mode}: launching {MISSIONS[mode]["launch"]}'
                  + (' with the Gazebo window' if gazebo else ' headless'))
        cmd = ['ros2', 'launch', 'eyantra_kepler_colony', MISSIONS[mode]['launch'],
               f'gui:={"true" if gazebo else "false"}', 'rviz:=false']
        self.spawn('world', cmd)
        return True, 'Started.'

    def spawn(self, label, cmd):
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                             start_new_session=True, env=env, cwd=ROOT)
        with self.lock:
            self.procs.append((label, p))
        threading.Thread(target=self.read, args=(label, p), daemon=True).start()

    def read(self, label, p):
        mission = MISSIONS[self.mode]
        for line in p.stdout:
            line = line.rstrip()
            if not line:
                continue
            if label != 'world' or any(k in line for k in ('[launch.user]', 'ERROR', 'error')):
                self.note(f'[{label}] {line[-220:]}')
            if label == 'world' and WORLD_READY in line and self.phase == 'starting':
                with self.lock:
                    self.phase, self.message = 'running', 'World ready. Robot nodes started.'
                for n in mission['nodes']:
                    self.note(f'Starting {n}')
                    self.spawn(n, ['python3', os.path.join(SCRIPTS, n)])
            if mission['done'] in line and self.phase == 'running':
                with self.lock:
                    self.phase, self.message, self.finished = 'done', 'Mission complete.', time.time()
        if self.phase in ('starting', 'running') and label == 'world':
            with self.lock:
                self.phase, self.message, self.finished = 'error', 'The simulated world stopped unexpectedly.', time.time()

    def stop(self):
        with self.lock:
            procs, self.procs = self.procs, []
            if not procs:
                self.phase, self.message = 'idle', 'Nothing was running.'
                return
            self.phase, self.message = 'stopping', 'Stopping...'
        self.note('Stopping the robot nodes and the simulated world.')
        for label, p in reversed(procs):            # nodes first, then the world
            self.signal(p, signal.SIGINT)
        deadline = time.time() + 10
        for _, p in procs:
            try:
                p.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                self.signal(p, signal.SIGKILL)
        # Gazebo's server can outlive its launch; end whatever is left in our sessions.
        sessions = {p.pid for _, p in procs}
        out = subprocess.run(['ps', '-eo', 'pid=,sid='], capture_output=True, text=True).stdout
        for row in out.splitlines():
            pid, sid = (int(v) for v in row.split())
            if sid in sessions:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        with self.lock:
            self.phase, self.message = 'idle', 'Stopped. Ready for the next mission.'
            self.finished = self.finished or time.time()
        self.note('Stopped.')

    @staticmethod
    def signal(p, sig):
        try:
            os.killpg(os.getpgid(p.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass


##################### ROS LISTENER #######################

class DashboardNode(Node):
    '''Listens to the robots and keeps the latest of everything the page shows.'''

    def __init__(self):
        super().__init__('strata_cobot_dashboard')
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.imagecb, qos_profile_sensor_data)
        self.create_subscription(String, '/ore_perception/status', lambda m: self.jsoncb('perception', m), 10)
        self.create_subscription(String, '/ore_sorter/status', lambda m: self.jsoncb('sorter', m), 10)
        self.create_subscription(String, '/path_planning/status', lambda m: self.jsoncb('planning', m), 10)
        self.create_subscription(Float32, '/ur7e/end_effector/force_magnitude', self.forcecb, 10)
        self.create_subscription(Int32, '/arm_status', self.statuscb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.mapcb, latched)
        self.create_subscription(Path, '/ebot_path', self.pathcb, latched)
        self.create_subscription(Odometry, '/odom', self.odomcb, 10)
        self.reset(None)

    def reset(self, mode):
        with self.lock:
            self.image_msg = None
            self.data = {'perception': None, 'sorter': None, 'planning': None}
            self.force = collections.deque(maxlen=200)
            self.last_force_t = 0.0
            self.arm_status = None
            self.map_png, self.map_info = None, None
            self.paths, self.last_path_stamp = [], None
            self.odom = None
            self.trail = collections.deque(maxlen=600)

    # ---- callbacks ----
    def imagecb(self, msg):
        self.image_msg = msg

    def jsoncb(self, key, msg):
        try:
            self.data[key] = json.loads(msg.data)
        except ValueError:
            pass

    def forcecb(self, msg):
        t = time.time()
        if t - self.last_force_t >= 0.1:                    # keep 10 samples a second
            self.last_force_t = t
            self.force.append(round(float(msg.data), 3))

    def statuscb(self, msg):
        self.arm_status = int(msg.data)

    def mapcb(self, msg):
        i = msg.info
        occ = np.array(msg.data, dtype=np.int16).reshape(i.height, i.width)
        step = max(1, int(round(0.02 / i.resolution)))        # draw at 2 cm a pixel
        small = occ[::step, ::step]
        img = np.full(small.shape + (3,), (236, 228, 220), np.uint8)
        img[small >= 50] = (60, 56, 52)
        img = np.ascontiguousarray(img[::-1])                  # +y up on the page
        ok, png = cv2.imencode('.png', img)
        with self.lock:
            self.map_png = png.tobytes() if ok else None
            self.map_info = {'origin': [i.origin.position.x, i.origin.position.y], 'res': i.resolution * step,
                             'w': int(small.shape[1]), 'h': int(small.shape[0])}

    def pathcb(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_path_stamp:
            return
        self.last_path_stamp = stamp
        with self.lock:
            self.paths.append([[round(p.pose.position.x, 3), round(p.pose.position.y, 3)] for p in msg.poses])

    def odomcb(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom = {'x': round(p.x, 3), 'y': round(p.y, 3), 'yaw': round(yaw, 3),
                     'v': round(math.hypot(t.linear.x, t.linear.y), 3), 'w': round(t.angular.z, 3)}
        if not self.trail or math.hypot(p.x - self.trail[-1][0], p.y - self.trail[-1][1]) > 0.03:
            self.trail.append((round(p.x, 3), round(p.y, 3)))

    # ---- for the page ----
    def snapshot(self):
        with self.lock:
            return {'perception': self.data['perception'], 'sorter': self.data['sorter'],
                    'planning': self.data['planning'], 'force': list(self.force), 'arm_status': self.arm_status,
                    'map': self.map_info, 'paths': self.paths, 'odom': self.odom, 'trail': list(self.trail)}

    def camera_jpeg(self):
        '''The latest camera frame with each detected ore outlined and labelled.'''
        msg = self.image_msg
        if msg is None:
            img = np.full((960, 1280, 3), 34, np.uint8)
            cv2.putText(img, 'Waiting for the camera...', (380, 490), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (200, 200, 200), 3)
        else:
            scale = 2                                           # draw at twice the size, so labels stay sharp
            img = cv2.resize(self.bridge.imgmsg_to_cv2(msg, 'bgr8'), None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_LINEAR)
            per = self.data['perception'] or {}
            font, size = cv2.FONT_HERSHEY_SIMPLEX, 0.7
            placed = []                                         # label rectangles already drawn
            faces = sorted(per.get('faces', []), key=lambda f: min(p[1] for p in f['box']))
            for f in faces:
                box = (np.array(f['box']) * scale).astype(np.int32)
                m = f.get('mineral')
                col = MINERAL_BGR.get(m, (200, 200, 200))
                cv2.polylines(img, [box], True, col, 3)
                label = (m or 'uncertain').replace('_ore', '')
                share = max(f.get('shares', {}).values() or [0])
                text = f'{label} {share:.0%}' if m else label
                (tw, th), base = cv2.getTextSize(text, font, size, 2)
                x = int(np.clip(box[:, 0].mean() - tw / 2, 2, img.shape[1] - tw - 2))
                y = int(box[:, 1].min()) - 10
                for _ in range(12):                             # move up until it overlaps no other label
                    r = (x - 4, y - th - 4, x + tw + 4, y + base + 2)
                    if all(r[2] < q[0] or r[0] > q[2] or r[3] < q[1] or r[1] > q[3] for q in placed):
                        break
                    y -= th + 10
                placed.append((x - 4, y - th - 4, x + tw + 4, y + base + 2))
                cv2.rectangle(img, (x - 4, y - th - 4), (x + tw + 4, y + base + 2), (20, 22, 24), cv2.FILLED)
                cv2.line(img, (int(box[:, 0].mean()), int(box[:, 1].min())), (int(box[:, 0].mean()), y + base + 2), col, 1)
                cv2.putText(img, text, (x, y), font, size, col, 2, cv2.LINE_AA)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return jpg.tobytes()


##################### WEB SERVER #######################

def make_handler(node, runner):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, code, body, ctype):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ('/', '/index.html'):
                return self.static('index.html')
            if path.startswith('/static/'):
                return self.static(path[len('/static/'):])
            if path == '/events':
                return self.events()
            if path == '/camera.mjpg':
                return self.camera()
            if path == '/map.png':
                png = node.map_png
                return self.send(200, png, 'image/png') if png else self.send(404, b'no map yet', 'text/plain')
            self.send(404, b'not found', 'text/plain')

        def do_POST(self):
            path = urlparse(self.path).path
            n = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(n) or b'{}')
            if path == '/api/run':
                mode = body.get('mode')
                if mode not in MISSIONS:
                    return self.send(400, b'{"ok": false, "message": "unknown mission"}', 'application/json')
                ok, msg = runner.start(mode, bool(body.get('gazebo')))
                return self.send(200, json.dumps({'ok': ok, 'message': msg}).encode(), 'application/json')
            if path == '/api/stop':
                threading.Thread(target=runner.stop, daemon=True).start()
                return self.send(200, b'{"ok": true}', 'application/json')
            self.send(404, b'not found', 'text/plain')

        def static(self, name):
            full = os.path.normpath(os.path.join(STATIC, name))
            if not full.startswith(STATIC) or not os.path.isfile(full):
                return self.send(404, b'not found', 'text/plain')
            ctype = {'.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css',
                     '.svg': 'image/svg+xml', '.png': 'image/png'}.get(os.path.splitext(full)[1], 'application/octet-stream')
            with open(full, 'rb') as f:
                self.send(200, f.read(), ctype + '; charset=utf-8' if ctype.startswith('text') else ctype)

        def events(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                while True:
                    snap = {'run': runner.snapshot(), **node.snapshot()}
                    self.wfile.write(b'data: ' + json.dumps(snap).encode() + b'\n\n')
                    self.wfile.flush()
                    time.sleep(0.25)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def camera(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                while True:
                    jpg = node.camera_jpeg()
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(jpg)).encode() + b'\r\n\r\n' + jpg + b'\r\n')
                    self.wfile.flush()
                    time.sleep(0.12)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--port', type=int, default=8765)
    ap.add_argument('--no-browser', action='store_true', help='do not open the page automatically')
    args = ap.parse_args()

    url = f'http://localhost:{args.port}/'
    with socket.socket() as probe:                      # already running? then just show it
        if probe.connect_ex(('127.0.0.1', args.port)) == 0:
            print(f'The dashboard is already running on {url}', flush=True)
            if not args.no_browser:
                webbrowser.open(url)
            return

    # Ctrl+C and closing the terminal are handled here, not by ROS, so that quitting always
    # stops the simulation and the robot nodes first.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DashboardNode()
    runner = MissionRunner(node.reset)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(node, runner))
    server.daemon_threads = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    print(f'Strata Cobot dashboard on {url}  (Ctrl+C to quit)', flush=True)
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    finally:
        print('Stopping...', flush=True)
        runner.stop()
        server.server_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

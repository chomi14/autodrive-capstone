"""Camera parking viewer and stopped-state calibration controls."""
import json
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image as PillowImage, ImageTk
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.srv import SetParametersAtomically
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .configuration import CAPTURE_FIELDS, TUNING_TOPIC
from .tuning_panel import TuningPanel


ROLES = ('front', 'rear')
VIEWS = ('oriented', 'segmentation', 'bev')


class CameraParkingViewer(Node):
    def __init__(self):
        super().__init__('perpendicular_camera_viewer')
        self.declare_parameter('preview', True)
        self.preview = self.get_parameter('preview').value
        self.bridge = CvBridge()
        self.images = {}
        self.status = {}
        self.snapshot = None
        self.dirty = False
        self.pending_revision = None
        self.challenge = ''
        self.closed = False
        self.last_heartbeat = 0.0
        self.preview_pub = self.create_publisher(Bool, '/parking_camera/preview_enabled', 1)
        self.key_pub = self.create_publisher(String, 'vehicle/drive_key', 1)
        self.heartbeat_pub = self.create_publisher(String, 'vehicle/tuner_heartbeat', 1)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'vehicle/arm_challenge', self.on_challenge, latched)
        self.create_subscription(String, '/parking_camera/status', self.on_status, 1)
        self.create_subscription(String, TUNING_TOPIC, self.on_tuning, latched)
        for role in ROLES:
            for view in VIEWS:
                key = role, view
                self.create_subscription(Image, '/parking_camera/' + '/'.join(key),
                                         lambda msg, key=key: self.on_image(key, msg), qos_profile_sensor_data)
        self.reset_client = self.create_client(Trigger, '/perpendicular_camera_controller/reset')
        self.save_client = self.create_client(Trigger, '/perpendicular_camera_controller/save')
        self.apply_client = self.create_client(
            SetParametersAtomically, 'perpendicular_camera_controller/set_parameters_atomically')
        self.build_window()
        self.timer = self.create_timer(.08, self.draw)

    def build_window(self):
        self.root = tk.Tk()
        self.root.title('Perpendicular parking | ' + ('PREVIEW' if self.preview else 'LIVE'))
        self.root.geometry('1260x760')
        self.root.minsize(840, 520)
        self.root.protocol('WM_DELETE_WINDOW', lambda: self.handle_key('q'))
        self.root.bind('<KeyPress>', self.on_key)
        toolbar = ttk.Frame(self.root, padding=(8, 6))
        toolbar.pack(fill='x')
        for label, key in [('Start (W)', 'w'), ('Stop (S / Esc)', 's'), ('Reset (R)', 'r')]:
            ttk.Button(toolbar, text=label, command=lambda key=key: self.handle_key(key)).pack(side='left', padx=(0, 6))
        ttk.Button(toolbar, text='Apply', command=self.apply).pack(side='left', padx=(0, 6))
        ttk.Button(toolbar, text='Reload', command=self.reload).pack(side='left', padx=(0, 6))
        ttk.Button(toolbar, text='Save YAML (P)', command=lambda: self.handle_key('p')).pack(side='left')
        self.feedback = tk.StringVar(value='Waiting for controller settings')
        ttk.Label(self.root, textvariable=self.feedback, padding=(8, 2)).pack(fill='x')
        self.summary = tk.StringVar(value='Waiting for perception')
        ttk.Label(self.root, textvariable=self.summary, padding=(8, 4), justify='left').pack(fill='x')
        body = ttk.Frame(self.root)
        body.pack(fill='both', expand=True, padx=8, pady=(0, 8))
        settings = ttk.Frame(body, width=410)
        settings.pack(side='right', fill='y', padx=(8, 0))
        settings.pack_propagate(False)
        self.video = tk.Canvas(body, background='#141414', highlightthickness=0)
        self.video.pack(side='left', fill='both', expand=True)
        self.video_image = self.video.create_image(0, 0, anchor='nw')
        self.panel = TuningPanel(settings, self.mark_dirty)

    def mark_dirty(self):
        self.dirty = True
        self.feedback.set('Unapplied settings. Stop and apply, or reload.')

    def on_challenge(self, message):
        self.challenge = message.data

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_tuning(self, message):
        snapshot = json.loads(message.data)
        if self.snapshot is not None and snapshot['revision'] != self.snapshot['revision']:
            self.images.clear()
        self.snapshot = snapshot
        if self.pending_revision is not None and snapshot['revision'] >= self.pending_revision:
            self.pending_revision = None
            self.reload()
            self.feedback.set('Applied. Waiting for fresh camera frames. Save YAML to keep these settings.')
        elif not self.dirty and self.pending_revision is None:
            self.reload()

    def reload(self):
        if self.snapshot is None or self.pending_revision is not None:
            return
        self.panel.load(self.snapshot)
        self.dirty = False
        self.feedback.set('Stop before applying settings. Save YAML to keep applied settings.')

    def on_image(self, key, message):
        self.images[key] = (self.bridge.imgmsg_to_cv2(message, 'bgr8'), time.monotonic())

    def on_key(self, event):
        if event.keysym == 'Escape':
            self.handle_key('s')
            return 'break'
        if isinstance(event.widget, (ttk.Entry, ttk.Combobox)) or event.char == ' ':
            return
        self.handle_key(event.char.lower())

    def handle_key(self, key):
        if key == 'w' and (self.pending_revision is not None or self.dirty):
            self.feedback.set('Apply or reload draft settings before starting.')
            return
        if key == 'w' and self.status.get('capture_restart_pending') and not self.preview:
            self.feedback.set('Save YAML and restart to use the new camera capture settings.')
            return
        if key in ('w', 's', 'x', ' ', 'q'):
            if self.preview:
                self.preview_pub.publish(Bool(data=key == 'w'))
            elif key == 'w':
                self.heartbeat_pub.publish(String(data=self.challenge))
                self.key_pub.publish(String(data='w:' + self.challenge))
            else:
                self.key_pub.publish(String(data='s'))
        if key == 'r':
            self.call_trigger(self.reset_client)
        if key == 'p':
            if self.dirty or self.pending_revision is not None:
                self.feedback.set('Apply the draft before saving.')
            else:
                self.call_trigger(self.save_client)
        if key == 'q':
            self.closed = True

    def call_trigger(self, client):
        if not client.service_is_ready():
            self.feedback.set('Controller service is not ready.')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self.feedback.set(f.result().message if f.exception() is None else str(f.exception())))

    def apply(self):
        if self.snapshot is None or self.pending_revision is not None:
            return
        if self.status.get('armed') is not False:
            self.feedback.set('Press Stop before applying settings.')
            return
        if not self.apply_client.service_is_ready():
            self.feedback.set('Controller service is not ready.')
            return
        try:
            changes = self.panel.changes()
        except ValueError as exc:
            self.feedback.set(str(exc))
            return
        if not any(changes.values()):
            self.reload()
            return
        revision = self.panel.snapshot['revision']
        request = SetParametersAtomically.Request()
        camera_edit = json.dumps({'revision': revision,
                                  'cameras': {role: changes[role] for role in ROLES}}, allow_nan=False)
        parameters = [Parameter('camera_tuning_json', value=camera_edit)]
        parameters.extend(Parameter(key, value=value) for key, value in changes['control'].items())
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        self.pending_revision = revision + 1
        self.feedback.set('Applying settings…')
        future = self.apply_client.call_async(request)
        future.add_done_callback(self.on_applied)

    def on_applied(self, future):
        try:
            result = future.result().result
            if result.successful:
                return  # the latched snapshot confirms which settings are active
            reason = result.reason
        except Exception as exc:
            reason = str(exc)
        self.pending_revision = None
        self.feedback.set(reason)

    def draw_panel(self, panel, role, view):
        sample = self.images.get((role, view))
        label = f'{role.upper()} / {view.upper()}'
        if sample is not None:
            image, arrived = sample
            h, w = panel.shape[:2]
            ih, iw = image.shape[:2]
            scale = min(w / iw, (h-24) / ih)
            nw, nh = max(1, int(iw * scale)), max(1, int(ih * scale))
            x, y = (w-nw)//2, 24 + (h-24-nh)//2
            frame = cv2.resize(image, (nw, nh))
            if self.snapshot and view in ('oriented', 'bev'):
                cfg = self.snapshot['config']
                point_key = 'persp_src' if view == 'oriented' else 'persp_dst'
                corners = np.round(np.array(cfg[role][point_key]).reshape(4, 2) * [nw-1, nh-1]).astype(np.int32)
                cv2.polylines(frame, [corners], True, (255, 230, 50), 1)
                for index, (px, py) in enumerate(corners, 1):
                    cv2.circle(frame, (px, py), 4, (255, 230, 50), -1)
                    cv2.putText(frame, str(index), (min(px+5, nw-11), max(12, min(py, nh-4))),
                                cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 230, 50), 1)
                if view == 'bev':
                    limits = [('outline_finish_y', (60, 190, 255))] if role == 'front' else [
                        ('end_steering_y', (60, 190, 255)), ('end_stop_y', (70, 80, 255))]
                    for key, color in limits:
                        row = round(cfg['control'][key] * (nh-1))
                        cv2.line(frame, (0, row), (nw-1, row), color, 1)
                        cv2.putText(frame, key, (4, max(12, row-4)), cv2.FONT_HERSHEY_SIMPLEX, .35, color, 1)
            panel[y:y+nh, x:x+nw] = frame
            age = time.monotonic()-arrived
            label += f'  {iw}x{ih}  {age:.2f}s'
            if age > .5:
                label += ' STALE'
        else:
            label += '  WAITING'
        cv2.putText(panel, label, (5, 16), cv2.FONT_HERSHEY_SIMPLEX, .38, (235, 235, 235), 1)

    def draw(self):
        w, h = self.video.winfo_width()//3, self.video.winfo_height()//2
        if w >= 25 and h >= 45:
            canvas = np.full((h*2, w*3, 3), 20, np.uint8)
            for row, role in enumerate(ROLES):
                for col, view in enumerate(VIEWS):
                    self.draw_panel(canvas[row*h:(row+1)*h, col*w:(col+1)*w], role, view)
            self.photo = ImageTk.PhotoImage(PillowImage.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)))
            self.video.itemconfigure(self.video_image, image=self.photo)
        s = self.status
        obs = s.get('observations', {})
        rear = obs.get('rear') or {}
        front = obs.get('front') or {}
        end, outline = rear.get('end', {}), front.get('outline', {})
        end_y = f"{end['y']:.3f}" if end.get('found') else '—'
        exit_y = f"{outline['y']:.3f}" if outline.get('found') else '—'
        lines = [
            f"{'PREVIEW' if self.preview else 'LIVE'}  {s.get('state', 'WAITING')}  {s.get('reason', '')}     "
            f"Armed {s.get('armed', '—')}   Steer {s.get('steering', '—')}   PWM {s.get('pwm', '—')}",
            f"Seen / missing / aligned  {s.get('seen_count', 0)} / {s.get('missing_count', 0)} / {s.get('align_count', 0)}"
            f"     End y {end_y}    Exit y {exit_y}    Errors {s.get('errors', {})}",
        ]
        if self.snapshot:
            active = self.snapshot['active_capture']
            lines.append('Capture  ' + '  |  '.join(f"{role} {active[role]['device']} "
                         f"{active[role]['width']}×{active[role]['height']} @ {active[role]['fps']:g}" for role in ROLES))
            if any(self.snapshot['config'][role][key] != active[role][key] for role in ROLES for key in CAPTURE_FIELDS):
                lines.append('Capture changes pending. Save YAML and restart using that file.')
        self.summary.set('\n'.join(lines))
        now = time.monotonic()
        if not self.closed and self.challenge and now-self.last_heartbeat >= .1:
            self.heartbeat_pub.publish(String(data=self.challenge))
            self.last_heartbeat = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraParkingViewer()
        while rclpy.ok() and not node.closed:
            node.root.update()
            rclpy.spin_once(node, timeout_sec=.01)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.handle_key('s')
            node.root.destroy()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

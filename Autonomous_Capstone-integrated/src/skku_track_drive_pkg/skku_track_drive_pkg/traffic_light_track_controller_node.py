"""Pure track driving with a traffic-light stop/go overlay."""

import json

import cv2
from std_msgs.msg import String

from .controller_liveness import run_controller
from .mode_parameters import TRAFFIC_LIGHT, limits, trackbars
from .track_controller_node import TrackControllerNode
from .traffic_light_core import TrafficLightCore


class TrafficLightTrackControllerNode(TrackControllerNode):
    def __init__(self):
        super().__init__('traffic_light_track_controller_node')

    def declare_mode_parameters(self):
        for name, (default, *_rest) in TRAFFIC_LIGHT.items():
            self.declare_parameter(name, default)

    def initialize_mode(self):
        raw_names = self.yolo.yolo.names
        names = set(raw_names.values() if hasattr(raw_names, 'values') else raw_names)
        if 'traffic_light' not in names:
            raise RuntimeError(
                f'Traffic-light track mode requires class traffic_light; names={sorted(names)}'
            )
        self.traffic_light = TrafficLightCore({
            name: self.get_parameter(name).value for name in TRAFFIC_LIGHT
        })
        self.status_pub = self.create_publisher(String, '/traffic_light_track/status', 1)
        self.get_logger().info(
            f'Traffic-light track model classes verified: {sorted(names)}'
        )

    @staticmethod
    def _tuning_limits():
        return {**TrackControllerNode._tuning_limits(), **limits(TRAFFIC_LIGHT)}

    def on_tuning_parameters(self, parameters):
        merged = dict(self.traffic_light.p)
        merged.update({p.name: p.value for p in parameters if p.name in TRAFFIC_LIGHT})
        if (
            merged['green_h_min'] > merged['green_h_max']
            or merged['yellow_h_min'] > merged['yellow_h_max']
        ):
            from rcl_interfaces.msg import SetParametersResult
            return SetParametersResult(
                successful=False,
                reason='HSV minimum must not exceed maximum',
            )
        return super().on_tuning_parameters(parameters)

    def _apply_tuning_values(self, updates):
        super()._apply_tuning_values(updates)
        self.traffic_light.p.update({
            key: value for key, value in updates.items() if key in TRAFFIC_LIGHT
        })
        # Turning the wait switch on is an explicit stop request. Turning it
        # off must not silently release an already confirmed red latch.
        if updates.get('wait_for_green') == 1:
            self.traffic_light.red_latched = True

    def adjust_command(self, command, detections, path, frame):
        stop, limit = self.traffic_light.decide(detections, frame)
        self.status_pub.publish(String(data=json.dumps(self.traffic_light.status)))
        return self.traffic_light.apply(command, stop, limit)

    def _make_debug(self, frame, detections, *args):
        marked = frame.copy()
        status = self.traffic_light.status
        for evidence in status.get('traffic_boxes', []):
            x1, y1, x2, y2 = map(int, evidence['box'])
            color = (255, 255, 0) if evidence.get('eligible') else (128, 128, 128)
            cv2.rectangle(marked, (x1, y1), (x2, y2), color, 2)
            label = evidence.get('color', evidence.get('rejection', 'Unknown'))
            ratios = evidence.get('ratios', {})
            text = (
                f"{label} used={evidence.get('used_for_color')} R/G/Y="
                + '/'.join(f"{ratios.get(name, 0):.2f}" for name in ('Red', 'Green', 'Yellow'))
            )
            cv2.putText(
                marked,
                text,
                (max(0, x1), max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                color,
                1,
            )
        debug = super()._make_debug(marked, detections, *args)

        # A large, fixed panel makes the current decision visible without
        # reading terminal logs or inspecting individual bounding boxes.
        detected = status.get('traffic_color', 'Unknown')
        display_colors = {
            'Red': (0, 0, 255),
            'Yellow': (0, 255, 255),
            'Green': (0, 255, 0),
            'Unknown': (128, 128, 128),
        }
        panel_color = display_colors.get(detected, display_colors['Unknown'])
        panel_width = min(430, debug.shape[1])
        panel_left = debug.shape[1] - panel_width
        panel_bottom = min(104, debug.shape[0] - 1)
        cv2.rectangle(
            debug,
            (panel_left, 0),
            (debug.shape[1] - 1, panel_bottom),
            (20, 20, 20),
            -1,
        )
        cv2.rectangle(
            debug,
            (panel_left + 10, 10),
            (panel_left + 82, min(82, panel_bottom - 10)),
            panel_color,
            -1,
        )
        cv2.putText(
            debug,
            f"TRAFFIC LIGHT: {detected.upper()}",
            (panel_left + 96, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            panel_color,
            2,
        )
        action = (
            'STOP'
            if status.get('red_latched')
            else 'SLOW'
            if detected == 'Yellow'
            else 'GO'
        )
        cv2.putText(
            debug,
            f"ACTION: {action}  RED LATCH: {status.get('red_latched')}",
            (panel_left + 96, 59),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            debug,
            f"R {status.get('red_count', 0)}/{status.get('red_confirm_required', 0)}  "
            f"G {status.get('green_count', 0)}/{status.get('green_confirm_required', 0)}",
            (panel_left + 96, 84),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
        )
        return debug


def main(args=None):
    run_controller(TrafficLightTrackControllerNode, args)


def tuner_main(args=None):
    import rclpy
    from .track_tuner_node import TrackTunerNode

    rclpy.init(args=args)
    node = None
    try:
        node = TrackTunerNode(
            node_name='traffic_light_track_tuner_node',
            extra_trackbars=trackbars(TRAFFIC_LIGHT),
            parameter_root='traffic_light_track_controller_node',
            window_prefix='Traffic Light Track',
        )
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

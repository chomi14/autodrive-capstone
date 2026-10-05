#!/usr/bin/env python3
"""Same sequential frames + common tuning, production track pipeline, no I/O.

Mission decisions are excluded deliberately to compare normal lane2 driving.
Agreement is not accuracy: there is no annotated ground truth here.
"""
import argparse
import csv
import hashlib
import gzip
import json
import os
from pathlib import Path
import time
from unittest.mock import Mock

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from launch import LaunchContext
from vehicle_bringup_pkg.tuning_configuration import resolve_tuning, tuning_launch_arguments
from skku_track_drive_pkg.track_controller_node import TrackControllerNode
from skku_track_drive_pkg.mission_core import lane_mask
from skku_track_drive_pkg.tuning_analysis import path_change


ROOT = Path(__file__).resolve().parents[1]


def settings(mode):
    context = LaunchContext()
    for argument in tuning_launch_arguments(mode):
        context.launch_configurations[argument.name] = ''.join(s.perform(context) for s in argument.default_value)
    return resolve_tuning(context, mode)


def stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return {'count': 0}
    return dict(zip(('count', 'median', 'p95', 'max'), (len(values), *map(float, np.percentile(values, [50, 95, 100])))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video')
    parser.add_argument('--output', required=True)
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--max-frames', type=int, default=0)
    parser.add_argument('--save-masks', action='store_true')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '147':
        raise RuntimeError('Use ROS_DOMAIN_ID=147 for isolated model comparison')
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    common, selected, _, source = settings('track')
    mission_settings, mission_selected, _, mission_source = settings('mission')
    report = {'common_tuning': common, 'common_source': source, 'common_selected': str(selected),
              'mission_selected': str(mission_selected), 'mission_source': mission_source,
              'mission_resolved_speed': mission_settings['speed'],
              'parameters_changed': False, 'serial_sender_started': False,
              'meaning': 'model agreement on the same input, no ground truth or physical steering validation'}
    rclpy.init()
    nodes, models = [], {}
    try:
        paths = {'track': ROOT/'src/skku_track_drive_pkg/models/best.pt',
                 'mission': ROOT/'src/skku_track_drive_pkg/models/mission/best.pt', 'root': ROOT/'best.pt'}
        for name in ('track', 'mission'):
            parameters = {**common, 'model_path': str(paths[name]), 'device': args.device, 'start_enabled': True,
                          'cmd_topic': '/dry_run/comparison/' + name, 'publish_debug': False, 'publish_bev_debug': False,
                          'publish_analysis': True, 'max_steering': 7.0, 'steering_sign': 1.0}
            rosargs = ['--ros-args']
            for key, value in parameters.items():
                rosargs.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
            # The node's own default fills advanced settings not in YAML.
            # Node constructor reads the global arguments; separate init contexts
            # are unnecessary: temporary explicit CLI override via Node.__init__.
            from unittest.mock import patch
            from rclpy.node import Node
            original = Node.__init__
            def initialize(node, node_name, *a, **kw):
                original(node, node_name, *a, cli_args=rosargs, use_global_arguments=False, **kw)
            with patch.object(Node, '__init__', initialize):
                node = TrackControllerNode('comparison_' + name)
            nodes.append(node)
            node.timing_pub.publish = Mock()
            original_detect = node.yolo.detect
            def capture(frame, detect=original_detect, n=node):
                result = detect(frame)
                n.comparison_mask = lane_mask(result, 'lane2', frame.shape)
                return result
            node.yolo.detect = capture
            models[name] = {'path': str(paths[name]), 'sha256': hashlib.sha256(paths[name].read_bytes()).hexdigest(),
                            'classes': node.yolo.yolo.names, 'task': node.yolo.yolo.task}
        from ultralytics import YOLO
        root_model = YOLO(str(paths['root']))
        models['root'] = {'path': str(paths['root']), 'sha256': hashlib.sha256(paths['root'].read_bytes()).hexdigest(),
                          'classes': root_model.names, 'task': root_model.task}
        report['models'] = models
        report['mission_equals_root'] = models['mission']['sha256'] == models['root']['sha256']
        if not args.video:
            report['video'] = None
            (out/'summary.json').write_text(json.dumps(report, indent=2))
            return
        video = Path(args.video).expanduser().resolve()
        cap = cv2.VideoCapture(str(video))
        if not cap.isOpened():
            raise ValueError(f'Cannot open {video}')
        fps = cap.get(cv2.CAP_PROP_FPS)
        if not np.isfinite(fps) or fps <= 0:
            raise ValueError('Video needs valid source FPS for timestamp comparison')
        report['video'] = {'path': str(video), 'sha256': hashlib.sha256(video.read_bytes()).hexdigest(),
                           'fps': fps, 'frames_available': int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), 'resize_to': [640, 480]}
        bridge, rows = CvBridge(), []
        with gzip.open(out/'frames.jsonl.gz','wt',encoding='utf-8') as log:
            index = 0
            while not args.max_frames or index < args.max_frames:
                ok, frame = cap.read()
                if not ok:
                    break
                frame = cv2.resize(frame, (640, 480))
                if index == 0:
                    for node in nodes:
                        for _ in range(3):
                            node.yolo.detect(frame)  # CUDA warmup; no path state updates.
                records = []
                message = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                source_ns = round(index/fps * 1e9)
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(source_ns, 1000000000)
                for node in nodes:
                    node.timing_pub.publish.reset_mock()
                    started = time.monotonic()
                    node.on_image(message)
                    if not node.timing_pub.publish.called:
                        raise RuntimeError(f'Pipeline failed at frame {index}')
                    record = json.loads(node.timing_pub.publish.call_args.args[0].data)
                    record['callback_ms'] = (time.monotonic()-started)*1000
                    record['lane2_pixels'] = int(np.count_nonzero(node.comparison_mask))
                    record['lane2_mask_sha256'] = hashlib.sha256(node.comparison_mask.tobytes()).hexdigest()
                    records.append(record)
                a, b = (n.comparison_mask.astype(bool) for n in nodes)
                union = np.count_nonzero(a | b)
                overlap = float(np.count_nonzero(a & b)/union) if union else None
                row = {'frame_index': index, 'video_time_s': index/fps, 'lane2_mask_iou_agreement': overlap,
                       'steering_delta_steps': records[1]['steering']-records[0]['steering'],
                       'cte_delta_px': records[1]['cte_px']-records[0]['cte_px'],
                       'heading_delta_deg': (records[1]['heading_error_deg']-records[0]['heading_error_deg']+180)%360-180,
                       **path_change(records[0]['path_points_bev_px'], records[1]['path_points_bev_px'])}
                rows.append(row)
                log.write(json.dumps({**row, 'track': records[0], 'mission_model_normal': records[1]})+'\n')
                if args.save_masks or index % 500 == 0:
                    for name, node in zip(('track','mission'), nodes):
                        cv2.imwrite(str(out/f'{index:06d}_{name}_lane2.png'), node.comparison_mask*255)
                    cv2.imwrite(str(out/f'{index:06d}_input.jpg'), frame)
                if index % 500 == 0:
                    print('Compared frame', index, flush=True)
                index += 1
        cap.release()
        report['frames_compared'] = len(rows)
        report['frame_records'] = 'frames.jsonl.gz'
        report['agreement'] = {key: stats([abs(r[key]) if r[key] is not None and key != 'lane2_mask_iou_agreement' else r[key] for r in rows])
                               for key in ('lane2_mask_iou_agreement','steering_delta_steps','cte_delta_px','heading_delta_deg','path_delta_mean_px','path_delta_max_px')}
        report['model_results'] = {}
        with gzip.open(out/'frames.jsonl.gz','rt',encoding='utf-8') as stream:
            full = [json.loads(line) for line in stream]
        for label in ('track', 'mission_model_normal'):
            values = [r[label] for r in full]
            report['model_results'][label] = {'current_lane_visible_frames': sum(r['lane_currently_visible'] for r in values),
                                            'valid_path_frames': sum(r['path_valid'] for r in values),
                                            'valid_error_reference_frames': sum(r['error_reference_valid'] for r in values),
                                            'inference_ms': stats([r['inference_duration_s']*1000 for r in values]),
                                            'pipeline_ms': stats([r['pipeline_duration_s']*1000 for r in values]),
                                            'callback_ms': stats([r['callback_ms'] for r in values])}
        with (out/'comparison.csv').open('w',newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['frame_index'])
            writer.writeheader(); writer.writerows(rows)
        (out/'summary.json').write_text(json.dumps(report, indent=2))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3,1,sharex=True,figsize=(12,8))
        for ax, field, unit in zip(axes, ('steering','cte_px','heading_error_deg'), ('steps','BEV px','deg')):
            for label in ('track','mission_model_normal'):
                ax.plot([r['video_time_s'] for r in full], [r[label][field] for r in full], label=label, linewidth=.8)
            ax.set_ylabel(unit); ax.legend(); ax.grid(alpha=.3)
        axes[-1].set_xlabel('source video seconds; model agreement, no ground truth')
        fig.tight_layout(); fig.savefig(out/'control_comparison.png'); plt.close(fig)
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Analyze source-stamped control.csv; retain frames around lane transitions."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def analyze(path):
    records = [json.loads(row['details_json']) for row in csv.DictReader(Path(path).open())]
    result = {'rows': len(records), 'timing_ms': {}, 'states': {}, 'lane_changes': []}
    for name in ('pipeline_duration_s', 'inference_duration_s', 'analysis_build_ms'):
        values = [r[name]*(1 if name.endswith('_ms') else 1000) for r in records if r.get(name) is not None]
        result['timing_ms'][name] = dict(zip(('count','median','p95','max'),
            (len(values), *map(float,np.percentile(values,[50,95,100]))))) if values else {'count':0}
    for i, record in enumerate(records):
        state = record.get('mission_state', 'UNKNOWN')
        result['states'][state] = result['states'].get(state,0)+1
        if record.get('lane_switch_requested') or record.get('lane_switch_applied'):
            keep = ('frame_stamp_ns','path_lane','target_lane','path_points_bev_px','cte_px','heading_error_deg',
                    'steering','left_pwm','right_pwm','reason','change_from_previous','lane_switch_requested','lane_switch_applied')
            result['lane_changes'].append({'index':i,'before_current_after':[
                {key: r.get(key) for key in keep} for r in records[max(0,i-1):min(len(records),i+2)]]})
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv'); parser.add_argument('--output',required=True)
    args=parser.parse_args()
    Path(args.output).write_text(json.dumps(analyze(args.csv),indent=2))

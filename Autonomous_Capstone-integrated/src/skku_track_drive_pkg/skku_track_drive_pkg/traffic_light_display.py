"""Seyeon's traffic-light panel adapted for the combined local mission.

Display only: consumes MissionCore status; never changes decisions or commands.
Source: chomi14/autodrive-capstone, Seyeon commit 77d357c.
"""

import cv2
import numpy as np


def describe_traffic_boxes(frame, status, settings):
    """Sample raw camera pixels for telemetry only, including rejected boxes.

    The representative pixel is the median-brightness pixel in the most common
    hue bin among S/V-qualified pixels. If none qualify, show the brightest raw
    pixel so an overly high threshold can still be diagnosed. Hue is never
    averaged across the red 0/179 boundary.
    """
    result = []
    h, w = frame.shape[:2]
    for item in status.get('traffic_boxes', []):
        item = dict(item)
        x1, y1, x2, y2 = item['box']
        if not np.all(np.isfinite(item['box'])):
            result.append(item)
            continue
        left, top = int(np.clip(x1, 0, w)), int(np.clip(y1, 0, h))
        right, bottom = int(np.clip(x2, 0, w)), int(np.clip(y2, 0, h))
        crop = frame[top:bottom, left:right]
        if crop.size == 0:
            item['sample_reason'] = 'Empty / outside image'
            result.append(item)
            continue
        pixels = crop.reshape(-1, 3)
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        hue, saturation, value = hsv.T
        valid = (saturation >= settings['hsv_s_min']) & (value >= settings['hsv_v_min'])
        candidates = np.flatnonzero(valid)
        if candidates.size:
            peak = np.bincount(hue[candidates], minlength=180).argmax()
            candidates = candidates[hue[candidates] == peak]
            sample = candidates[np.argmin(np.abs(value[candidates].astype(float)
                                                  - np.median(value[candidates])))]
        else:
            sample = int(value.argmax())
        ratios = {
            'Red': float(np.mean(valid & ((hue <= settings['red_h_low_max']) | (hue >= settings['red_h_high_min'])))),
            'Green': float(np.mean(valid & (hue >= settings['green_h_min']) & (hue <= settings['green_h_max']))),
            'Yellow': float(np.mean(valid & (hue >= settings['yellow_h_min']) & (hue <= settings['yellow_h_max']))),
        }
        item.update(rgb_mean=np.rint(pixels.mean(axis=0)[::-1]).astype(int).tolist(),
                    rgb_sample=pixels[sample, ::-1].astype(int).tolist(),
                    hsv_sample=hsv[sample].astype(int).tolist(),
                    sv_valid_ratio=float(np.mean(valid)), sample_xy=[int(left + sample % crop.shape[1]),
                                                                   int(top + sample // crop.shape[1])])
        item.setdefault('ratios', ratios)
        if not item.get('eligible'):
            failures = []
            if x2 - x1 < settings['traffic_min_width'] * w / 640.:
                failures.append('bbox too narrow')
            if y2 > settings['traffic_max_bottom_y'] * h / 480.:
                failures.append('bbox bottom exceeds max Y')
            item['sample_reason'] = 'Ignored: ' + ', '.join(failures or ['width/bottom threshold'])
        elif not candidates.size:
            item['sample_reason'] = 'No pixels pass S/V (sample=brightest)'
        elif item.get('color') == 'Unknown':
            item['sample_reason'] = ('Color ratio below minimum' if max(ratios.values()) < settings['traffic_min_color_ratio']
                                     else 'No unique winning color')
        else:
            item['sample_reason'] = 'Used for decision' if item.get('used_for_color') else 'Other light has priority'
        result.append(item)
    return result


def add_traffic_light_panel(image, status, camera_frame=None):
    """Prepend a status strip, keeping camera/BEV pixels unobscured.

    GO describes the light decision only. The existing serial/arm status in
    the tuner remains the authority for whether the vehicle is enabled.
    """
    width = image.shape[1]
    boxes = status.get('traffic_boxes', [])
    columns = max(1, width // 640)
    shown = boxes[:4]
    rows = (len(shown) + columns - 1) // columns
    panel = np.full((168 + rows * 158, max(640, width), 3), 20, dtype=np.uint8)
    detected = status.get('traffic_color', 'Unknown')
    color = {
        'Red': (0, 0, 255),
        'Yellow': (0, 255, 255),
        'Green': (0, 255, 0),
    }.get(detected, (128, 128, 128))
    latched = status.get('red_latched')
    action = ('WAIT' if latched is None else 'STOP' if latched
              else 'SLOW' if detected == 'Yellow' else 'GO')
    latch_text = '--' if latched is None else 'ON' if latched else 'OFF'

    cv2.rectangle(panel, (12, 12), (80, 80), color, -1)
    cv2.putText(panel, f'TRAFFIC LIGHT: {detected.upper()}', (96, 28),
                cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2, cv2.LINE_AA)
    cv2.putText(panel, f'LIGHT ACTION: {action}   RED LATCH: {latch_text}',
                (96, 53), cv2.FONT_HERSHEY_SIMPLEX, .50,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(panel,
                f"R {status.get('red_count', 0)}/{status.get('red_confirm_required', '-')}   "
                f"G {status.get('green_count', 0)}/{status.get('green_confirm_required', '-')}",
                (96, 78), cv2.FONT_HERSHEY_SIMPLEX, .50,
                (255, 255, 255), 1, cv2.LINE_AA)
    if 'target_lane' in status:
        lane_text = f"Target: lane{status['target_lane']}"
        if 'lane_change_cooldown_remaining_s' in status:
            lane_text += (f"   Auto cooldown: {status['lane_change_cooldown_remaining_s']:.1f}s"
                          '   L: toggle lane')
        cv2.putText(panel, lane_text, (12, 102), cv2.FONT_HERSHEY_SIMPLEX,
                    .48, (0, 255, 255), 1, cv2.LINE_AA)
    note = ('No traffic_light bbox detected' if not boxes else
            'No eligible bbox: check size/position limits' if not any(b.get('eligible') for b in boxes)
            else 'RGB: 0..255 | OpenCV HSV: H 0..179, S/V 0..255')
    if len(boxes) > len(shown):
        note += f' | Showing {len(shown)}/{len(boxes)}; all in Tuner details'
    cv2.putText(panel, note, (12, 126), cv2.FONT_HERSHEY_SIMPLEX,
                .45, (220, 220, 220), 1, cv2.LINE_AA)
    limits = status.get('traffic_limits', {})
    if limits:
        text = (f"Limits: S>={limits['hsv_s_min']} V>={limits['hsv_v_min']} "
                f"color>={100*limits['traffic_min_color_ratio']:.1f}% "
                f"width>={limits['traffic_min_width']} bottom<={limits['traffic_max_bottom_y']} (640x480)")
        cv2.putText(panel, text, (12, 150), cv2.FONT_HERSHEY_SIMPLEX,
                    .42, (220, 220, 220), 1, cv2.LINE_AA)
    for index, item in enumerate(shown):
        x, y = (index % columns) * 640, 168 + (index // columns) * 158
        cv2.rectangle(panel, (x+4, y+2), (x+635, y+154), (65, 65, 65), 1)
        box = item['box']
        if camera_frame is not None and np.all(np.isfinite(box)):
            x1, y1, x2, y2 = map(int, box)
            h, w = camera_frame.shape[:2]
            crop = camera_frame[max(0,min(h,y1)):max(0,min(h,y2)), max(0,min(w,x1)):max(0,min(w,x2))]
            if crop.size:
                scale = min(96/crop.shape[1], 96/crop.shape[0])
                thumb = cv2.resize(crop, (max(1,round(crop.shape[1]*scale)), max(1,round(crop.shape[0]*scale))),
                                   interpolation=cv2.INTER_NEAREST)
                panel[y+30:y+30+thumb.shape[0], x+10:x+10+thumb.shape[1]] = thumb
        swatch = item.get('rgb_sample')
        if swatch is not None:
            cv2.rectangle(panel, (x+10,y+132), (x+104,y+146), tuple(reversed(swatch)), -1)
        ratios = item.get('ratios', {})
        lines = [f"TL#{index} {item.get('color', 'Rejected')}  eligible={item.get('eligible')}  used={item.get('used_for_color', False)}",
                 f"box w={box[2]-box[0]:.0f} bottom={box[3]:.0f}  {item.get('sample_reason', '')}",
                 f"ROI mean RGB: {item.get('rgb_mean', '--')}",
                 f"Sample RGB: {item.get('rgb_sample', '--')}  HSV: {item.get('hsv_sample', '--')}",
                 'R/G/Y: ' + '/'.join(f'{100*ratios.get(k,0):.1f}%' for k in ('Red','Green','Yellow'))
                 + f"  S/V pass: {100*item.get('sv_valid_ratio', 0):.1f}%"]
        for row, line in enumerate(lines):
            text_width = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, .43, 1)[0][0]
            scale = min(.43, .43 * 510 / max(1, text_width))
            cv2.putText(panel, line, (x+116, y+23+row*25), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, (230, 230, 230), 1, cv2.LINE_AA)
    if width < 640:
        panel = cv2.resize(panel, (width, max(1, round(panel.shape[0] * width / 640))),
                           interpolation=cv2.INTER_AREA)
    return np.vstack((panel, image))

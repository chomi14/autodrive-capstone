"""Editable camera and control settings inside the parking viewer."""
import math
import tkinter as tk
from tkinter import ttk

from .configuration import CAPTURE_FIELDS, REAR_VISION_FIELDS, VISION_FIELDS


CONTROL_GROUPS = {
    'Search and alignment': {
        'confirm_frames': 'Space detections (frames)',
        'space_missing_frames': 'Space lost (frames)',
        'align_confirm_frames': 'Alignment confirmation (frames)',
        'align_yaw_min_rad': 'Minimum alignment yaw (rad, 0 = legacy)',
        'align_yaw_error_max_rad': 'Legacy vertical tolerance (rad)',
        'align_steering': 'Alignment steering step',
        'direction_pause_s': 'Pause before reverse (s)',
        'front_target_x': 'Front target x (0–1)',
        'front_position_gain': 'Front position gain',
        'front_angle_gain': 'Front yaw gain',
    },
    'Reverse and stop': {
        'rear_target_x': 'Rear target x (0–1)',
        'rear_position_gain': 'Rear position gain',
        'rear_angle_gain': 'Rear yaw gain',
        'end_angle_gain': 'End line yaw gain',
        'end_steering_y': 'End line steering y (0–1)',
        'end_stop_y': 'End line stop y (0–1)',
    },
    'Exit': {
        'exit_enabled': 'Automatic exit',
        'stopped_hold_s': 'Parked hold (s)',
        'escape_s': 'Straight out (s)',
        'outline_finish_y': 'Exit line finish y (0–1)',
        'exit_position_gain': 'Exit position gain',
        'exit_angle_gain': 'Exit yaw gain',
        'final_straight_s': 'Final straight (s)',
    },
    'Vehicle and timing': {
        'forward_pwm': 'Forward PWM',
        'reverse_pwm': 'Reverse PWM magnitude',
        'exit_pwm': 'Exit PWM',
        'steering_sign': 'Steering sign (−1 or 1)',
        'max_steering': 'Maximum steering step',
        'steering_deadband': 'Steering deadband',
        'observation_timeout_s': 'Maximum frame age (s)',
        'observation_loss_fault_s': 'Missing observation timeout (s)',
        'search_timeout_s': 'Search timeout (s)',
        'maneuver_timeout_s': 'Maneuver timeout (s)',
        'calibration_confirmed': 'Vehicle calibration verified',
    },
}
CAMERA_LABELS = {
    'device': 'Capture device',
    'width': 'Capture width (px)',
    'height': 'Capture height (px)',
    'fps': 'Capture FPS',
    'rotate_deg': 'Clockwise rotation (degrees)',
    'flip_horizontal': 'Flip horizontally after rotation',
    'confidence': 'Segmentation confidence (0–1)',
    'persp_src': 'BEV source on camera image',
    'persp_dst': 'BEV destination on BEV image',
    'min_area_ratio': 'Minimum mask area (0–1)',
    'morph_kernel': 'Mask closing kernel (odd px)',
    'min_longitudinal_span_ratio': 'Minimum line height (0–1)',
    'roi_band_ratio': 'Rear boundary kernel (0 = entire mask)',
    'parking_width_ratio': 'Estimated slot width / BEV width',
    'pair_min_separation_ratio': 'Minimum side separation / BEV width',
    'pair_max_angle_rad': 'Maximum side angle difference (rad)',
}


class TuningPanel:
    def __init__(self, parent, on_change):
        self.book = ttk.Notebook(parent)
        self.book.pack(fill='both', expand=True)
        self.on_change = on_change
        self.loading = False
        self.snapshot = None
        self.fields = {}

    def _tab(self, title):
        tab = ttk.Frame(self.book)
        self.book.add(tab, text=title)
        canvas = tk.Canvas(tab, width=380, highlightthickness=0)
        bar = ttk.Scrollbar(tab, orient='vertical', command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        bar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)
        content = ttk.Frame(canvas, padding=6)
        window = canvas.create_window((0, 0), window=content, anchor='nw')
        content.bind('<Configure>', lambda _: canvas.configure(scrollregion=canvas.bbox('all')))
        canvas.bind('<Configure>', lambda event: canvas.itemconfigure(window, width=event.width))
        tab.bind('<Enter>', lambda _: canvas.bind_all('<Button-4>', lambda __: canvas.yview_scroll(-3, 'units')))
        tab.bind('<Enter>', lambda _: canvas.bind_all('<Button-5>', lambda __: canvas.yview_scroll(3, 'units')), add='+')
        tab.bind('<Leave>', lambda _: canvas.unbind_all('<Button-4>'))
        tab.bind('<Leave>', lambda _: canvas.unbind_all('<Button-5>'), add='+')
        return content

    def _group(self, parent, title, section, labels):
        group = ttk.LabelFrame(parent, text=title, padding=7)
        group.pack(fill='x', pady=(0, 7))
        group.columnconfigure(0, weight=1)
        for row, (key, label) in enumerate(labels.items()):
            value = self.snapshot['config'][section][key]
            if type(value) is bool:
                variable = tk.BooleanVar(value=value)
                ttk.Checkbutton(group, text=label, variable=variable).grid(row=row, column=0, columnspan=2, sticky='w', pady=2)
                variables = [variable]
            elif type(value) is list:
                points = ttk.LabelFrame(group, text=label, padding=4)
                points.grid(row=row, column=0, columnspan=2, sticky='ew', pady=3)
                variables = []
                for i, corner in enumerate(('1 bottom left', '2 bottom right', '3 top right', '4 top left')):
                    ttk.Label(points, text=corner).grid(row=i, column=0, sticky='w', padx=3)
                    for j in range(2):
                        variable = tk.StringVar(value=str(value[i*2+j]))
                        ttk.Entry(points, textvariable=variable, width=8).grid(row=i, column=j+1, padx=2, pady=1)
                        variables.append(variable)
            else:
                variable = tk.StringVar(value=str(value))
                ttk.Label(group, text=label).grid(row=row, column=0, sticky='w', padx=(0, 4), pady=2)
                if key == 'rotate_deg':
                    entry = ttk.Combobox(group, textvariable=variable, values=(0, 90, 180, 270),
                                         state='readonly', width=11)
                else:
                    entry = ttk.Entry(group, textvariable=variable, width=22 if key == 'device' else 13)
                entry.grid(row=row, column=1, sticky='e', pady=2)
                variables = [variable]
            self.fields[section, key] = variables
            for variable in variables:
                variable.trace_add('write', self._changed)

    def _changed(self, *_):
        if not self.loading:
            self.on_change()

    def load(self, snapshot):
        self.snapshot = snapshot
        self.loading = True
        if not self.fields:
            control = self._tab('Control')
            for title, labels in CONTROL_GROUPS.items():
                self._group(control, title, 'control', labels)
            for role in ('front', 'rear'):
                tab = self._tab(role.title())
                keys = VISION_FIELDS + (REAR_VISION_FIELDS if role == 'rear' else ())
                self._group(tab, 'Image and BEV', role, {key: CAMERA_LABELS[key] for key in keys})
                self._group(tab, 'Capture (save and restart)', role, {key: CAMERA_LABELS[key] for key in CAPTURE_FIELDS})
        else:
            for (section, key), variables in self.fields.items():
                value = snapshot['config'][section][key]
                for variable, item in zip(variables, value if isinstance(value, list) else [value]):
                    variable.set(item)
        self.loading = False

    def changes(self):
        changes = {section: {} for section in ('control', 'front', 'rear')}
        for (section, key), variables in self.fields.items():
            baseline = self.snapshot['config'][section][key]
            try:
                if isinstance(baseline, list):
                    value = [float(var.get()) for var in variables]
                    if not all(math.isfinite(point) for point in value):
                        raise ValueError('Non-finite point')
                else:
                    value = type(baseline)(variables[0].get())
                    if type(value) is float and not math.isfinite(value):
                        raise ValueError('Non-finite value')
            except (ValueError, tk.TclError) as exc:
                raise ValueError(f'{section}.{key}: enter a valid value') from exc
            if value != baseline:
                changes[section][key] = value
        return changes

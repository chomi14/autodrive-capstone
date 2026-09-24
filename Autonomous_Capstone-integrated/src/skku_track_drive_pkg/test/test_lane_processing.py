import numpy as np

from skku_track_drive_pkg import camera_perception_core as core
from skku_track_drive_pkg.lane_processing import LaneInfoExtractor
from skku_track_drive_pkg.messages import (
    Detection,
    DetectionArray,
    Mask,
    Point2D,
)


def test_segmentation_polygon_is_filled_not_only_outlined():
    detection = Detection(
        class_name='lane2',
        mask=Mask(
            data=[
                Point2D(10, 10),
                Point2D(50, 10),
                Point2D(50, 50),
                Point2D(10, 50),
            ],
            height=64,
            width=64,
        ),
    )

    mask = core.draw_edges(DetectionArray([detection]), 'lane2')

    assert mask[30, 30] == 255
    assert np.count_nonzero(mask) > 1500


def test_mask_cleanup_rejects_small_disconnected_noise():
    mask = np.zeros((120, 200), dtype=np.uint8)
    mask[:, 50:150] = 255
    mask[20:25, 180:185] = 255

    cleaned = core.clean_lane_mask(mask, min_component_area=100)

    assert cleaned[60, 100] == 255
    assert cleaned[22, 182] == 0


def test_single_edge_assignment_uses_history_not_slope_sign():
    mask = np.zeros((80, 200), dtype=np.uint8)
    mask[35:45, 98:103] = 255

    negative_slope = core.get_lane_center(
        mask, 40, 10, -0.5, 60, virtual_lane_width=60,
        previous_center=70,
    )
    positive_slope = core.get_lane_center(
        mask, 40, 10, 0.5, 60, virtual_lane_width=60,
        previous_center=70,
    )

    assert negative_slope.valid
    assert positive_slope.valid
    assert negative_slope.center == positive_slope.center
    assert negative_slope.center < 100


def test_missing_pixels_are_invalid_instead_of_fake_image_center():
    mask = np.zeros((80, 200), dtype=np.uint8)

    result = core.get_lane_center(mask, 40, 10, 0.0, 60)

    assert not result.valid
    assert result.source == 'invalid'


def test_target_filter_limits_jump_and_holds_short_miss():
    extractor = LaneInfoExtractor(
        show_image=False,
        center_ema_alpha=0.5,
        max_center_jump_px=40.0,
        max_missed_frames=2,
    )
    first = extractor._smooth_targets([100.0, 110.0, 120.0], [10, 20, 30])
    second = extractor._smooth_targets([400.0, 410.0, 420.0], [10, 20, 30])
    extractor.last_confidence = 0.9

    held = extractor._held_lane_info()

    assert first == [100.0, 110.0, 120.0]
    assert second == [120.0, 130.0, 140.0]
    assert held.valid
    assert held.source == 'held'
    assert [point.target_x for point in held.target_points] == [120, 130, 140]


"""Offline geometry and clipping checks, with no ROS nodes or actuator I/O."""
import cv2
import numpy as np
import pytest

from skku_track_drive_pkg.lane_processing import LaneInfoExtractor
from skku_track_drive_pkg.messages import Detection, DetectionArray, Mask
from skku_track_drive_pkg.path_planner import PathPlanner


def camera_detection(bitmap, lane='lane2'):
    return DetectionArray([Detection(
        class_name=lane, mask=Mask(bitmap=bitmap, width=640, height=480),
    )])


def detection_from_bev(extractor, bird, lane='lane2'):
    destination = np.float32([[192, 0], [448, 0], [448, 480], [192, 480]])
    inverse = cv2.getPerspectiveTransform(destination, np.float32(extractor._src_mat()))
    bitmap = cv2.warpPerspective(bird, inverse, (640, 480), flags=cv2.INTER_NEAREST)
    return camera_detection(bitmap, lane)


def stripe(left, right):
    bird = np.zeros((480, 640), np.uint8)
    bird[:, left:right] = 255
    return bird


@pytest.mark.parametrize('lane', ['lane1', 'lane2'])
@pytest.mark.parametrize('left,right', [(250, 390), (150, 300), (80, 185), (340, 490)])
def test_calibration_edges_do_not_crop_or_trigger_virtual_reconstruction(lane, left, right):
    extractor = LaneInfoExtractor(show_image=False, capture_debug=True)
    extractor.lane_class_name = lane
    detections = detection_from_bev(extractor, stripe(left, right), lane)

    info = extractor.process(detections)

    assert info.valid and info.source == 'observed'
    for point in info.target_points:
        assert abs(point.target_x - (left + right - 1) / 2) <= 2
    assert all(not row.reconstructed for _, row in extractor.last_debug['samples'])
    assert extractor.last_debug['row_counts'] == {'two_edges': 30}
    assert not np.any(extractor.last_debug['bev_mask'][:300])
    assert extractor.last_debug['bev_mask'].shape == (480, 640)
    assert np.all(extractor.last_debug['observed_mask'][350:450, left:right] == 255)


def test_corner_fit_and_spline_remain_in_existing_roi_coordinates():
    extractor = LaneInfoExtractor(show_image=False, capture_debug=True)
    bird = np.zeros((480, 640), np.uint8)
    for y in range(480):
        center = round(320 + .001 * (y - 300) ** 2)
        bird[y, center - 60:center + 60] = 255

    info = extractor.process(detection_from_bev(extractor, bird))
    path = PathPlanner().plan(info)

    assert info.valid
    assert [point.target_y for point in info.target_points] == [5, 55, 105]
    for point in info.target_points:
        assert abs(point.target_x - (320 + .001 * point.target_y ** 2)) <= 2
    assert len(path.x_points) == len(path.y_points) == 100
    assert path.y_points[0] == 5 and path.y_points[-1] == 105
    assert all(not row.reconstructed for _, row in extractor.last_debug['samples'])


@pytest.mark.parametrize('left,right,reason', [
    (0, 200, 'clipped_left'), (440, 640, 'clipped_right'),
])
def test_actual_canvas_clipping_still_reconstructs_in_corners(left, right, reason):
    extractor = LaneInfoExtractor(show_image=False, capture_debug=True)

    info = extractor.process(detection_from_bev(extractor, stripe(left, right)))

    assert info.valid
    assert extractor.last_debug['row_counts'][reason] >= 25
    assert all(row.reconstructed and row.reconstruction_reason == reason
               for _, row in extractor.last_debug['samples'])


def test_camera_coverage_boundary_can_be_inside_the_bev_canvas():
    # This homography maps the full camera width into a narrower BEV interval;
    # the observation border is independent of the calibration interval 192..448.
    extractor = LaneInfoExtractor(
        show_image=False, capture_debug=True, virtual_lane_width=120,
        bev_source_points=[[100, 0], [540, 0], [540, 479], [100, 479]],
    )

    info = extractor.process(camera_detection(stripe(0, 200)))

    assert info.valid
    coverage = np.flatnonzero(extractor.last_debug['observed_mask'][350])
    assert 0 < coverage[0] < 192
    assert 448 < coverage[-1] < 639
    assert extractor.last_debug['row_counts']['clipped_left'] == 30


def test_missing_detection_holds_then_expires_without_inventing_a_virtual_lane():
    extractor = LaneInfoExtractor(show_image=False, capture_debug=True, max_missed_frames=2)
    first = extractor.process(detection_from_bev(extractor, stripe(250, 390)))

    for _ in range(2):
        held = extractor.process(DetectionArray())
        assert held.valid and held.source == 'held'
        assert held.target_points == first.target_points
        assert extractor.last_debug['row_counts'] == {'missing': 30}
        assert extractor.last_debug['source'] == 'held'
        assert not extractor.last_debug['samples']
        assert np.any(extractor.last_debug['observed_mask'])
    assert not extractor.process(DetectionArray()).valid


def test_incomplete_interior_detection_is_distinguished_from_boundary_clipping():
    extractor = LaneInfoExtractor(show_image=False)
    result = extractor._row_center(stripe(300, 308), 40, 300, 150)

    assert result.valid and result.reconstructed
    assert result.source == 'right_edge'
    assert result.reconstruction_reason == 'incomplete_detection'


def test_unobserved_row_is_distinguished_from_a_missing_detection():
    extractor = LaneInfoExtractor(show_image=False)
    mask = np.zeros((80, 200), np.uint8)

    missing = extractor._row_center(mask, 40, 120, None, np.full_like(mask, 255))
    unobserved = extractor._row_center(mask, 40, 120, None, np.zeros_like(mask))

    assert not missing.valid and missing.source == 'missing'
    assert not unobserved.valid and unobserved.source == 'unobserved'


def test_both_boundaries_clipped_do_not_become_a_fake_observed_center():
    extractor = LaneInfoExtractor(show_image=False, capture_debug=True)
    first = extractor.process(detection_from_bev(extractor, stripe(250, 390)))

    info = extractor.process(camera_detection(np.full((480, 640), 255, np.uint8)))

    assert info.valid and info.source == 'held'
    assert info.target_points == first.target_points
    assert extractor.last_debug['row_counts'] == {'clipped_both': 30}


def test_pixels_projected_above_the_horizontal_roi_do_not_create_a_path():
    extractor = LaneInfoExtractor(show_image=False, capture_debug=True)
    bird = stripe(250, 390)
    bird[250:] = 0

    info = extractor.process(detection_from_bev(extractor, bird))

    assert not info.valid
    assert not np.any(extractor.last_debug['bev_mask'])


def test_foreign_blob_below_roi_is_not_proven_invalid_by_the_horizontal_cut():
    # A horizontal cut cannot distinguish a coherent false lane segmentation
    # from a real lane in the same pixels. Record this limitation explicitly.
    extractor = LaneInfoExtractor(show_image=False, capture_debug=True)
    bird = stripe(470, 580)
    bird[:300] = 0

    info = extractor.process(detection_from_bev(extractor, bird))

    assert info.valid and info.source == 'observed'
    assert all(point.target_x > 448 for point in info.target_points)

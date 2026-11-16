"""Unit tests for airmouse_detection.detector with a stub model (no torch needed)."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from airmouse_detection.detector import (SurvivorDetector, add_ranges,  # noqa: E402
                                         draw_detections, frame_record)


class _T:
    def __init__(self, v):
        self.v = v

    def tolist(self):
        return self.v


class _Box:
    def __init__(self, xyxy, conf):
        self.xyxy, self.conf = [_T(xyxy)], conf


class _Result:
    def __init__(self, boxes):
        self.boxes = boxes


class StubModel:
    """Stands in for ultralytics.YOLO: records the call and returns fixed boxes."""

    def __init__(self, boxes):
        self.boxes, self.kwargs = boxes, None

    def predict(self, frame, **kwargs):
        self.kwargs = kwargs
        return [_Result(self.boxes)]


def test_detect_uses_team_settings_and_spec_fields() -> None:
    """The model is called with person class, conf 0.403, imgsz 416; output follows spec v0.1."""
    stub = StubModel([_Box([10.0, 20.0, 50.0, 120.0], 0.87)])
    det = SurvivorDetector('unused.pt', model=stub)
    out = det.detect(np.zeros((240, 424, 3), np.uint8))
    assert stub.kwargs['classes'] == [0] and stub.kwargs['conf'] == 0.403
    assert stub.kwargs['imgsz'] == 416
    assert out == [{'det_id': 0, 'class_name': 'person', 'confidence': 0.87,
                    'bbox_xyxy': [10.0, 20.0, 50.0, 120.0], 'bbox_center': [30.0, 70.0]}]


def test_ranges_from_depth() -> None:
    """range_m is the median valid depth in the middle of the box; None without valid depth."""
    dets = [{'bbox_xyxy': [10, 20, 50, 120]}, {'bbox_xyxy': [300, 20, 340, 120]}]
    depth = np.full((240, 424), np.inf, np.float32)
    depth[40:100, 15:45] = 2.5
    add_ranges(dets, depth)
    assert dets[0]['range_m'] == 2.5 and dets[1]['range_m'] is None
    assert add_ranges([{'bbox_xyxy': [0, 0, 4, 4]}], None)[0]['range_m'] is None


def test_frame_record_and_drawing() -> None:
    """The record has the spec v0.1 keys; drawing returns a changed copy."""
    dets = [{'det_id': 0, 'class_name': 'person', 'confidence': 0.9,
             'bbox_xyxy': [10, 20, 50, 120], 'bbox_center': [30, 70], 'range_m': 1.8}]
    rec = frame_record(3, 'camera', 424, 240, 41.26, dets)
    assert set(rec) == {'frame_id', 'source', 'video_time_s', 'timestamp_utc', 'frame_width',
                        'frame_height', 'inference_ms', 'detections'}
    assert rec['timestamp_utc'].endswith('Z') and rec['inference_ms'] == 41.3
    frame = np.zeros((240, 424, 3), np.uint8)
    img = draw_detections(frame, dets)
    assert img.shape == frame.shape and img.any() and not frame.any()

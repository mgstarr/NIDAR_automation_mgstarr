"""Survivor detector core (no ROS imports): the detection team's model behind a small class.

This follows the usage in external/NIDAR-Automation/README.md section 7 ("Using the detector
today (v0)") and its detection-output spec v0.1 (cv/detectors/detect_survivors.py):

    det = SurvivorDetector(weights, conf=0.403, imgsz=416, device='cpu')
    detections = det.detect(frame_bgr)          # list of dicts, spec v0.1 fields:
        {"det_id": 0, "class_name": "person", "confidence": 0.87,
         "bbox_xyxy": [x1, y1, x2, y2], "bbox_center": [cx, cy]}
    record = frame_record(frame_id, source, w, h, inference_ms, detections)   # one JSONL line
    annotated = draw_detections(frame_bgr, detections)

Extra field added by this package (not in spec v0.1): "range_m", the median depth inside the
middle of the box, when a depth image is given to `add_ranges` (None if no valid depth).

Per-frame detection only: no tracking, no confirmation over frames (planned by the detection
team). Ultralytics runs offline (YOLO_OFFLINE=1): no network calls at runtime.
"""

from __future__ import annotations

import os
import time
from datetime import datetime, timezone

import numpy as np

PERSON_CLASS_ID = 0


class SurvivorDetector:
    """YOLO person detector. `model` may be injected for tests (anything with .predict)."""

    def __init__(self, weights: str, conf: float = 0.403, imgsz: int = 416,
                 device: str = 'cpu', model=None) -> None:
        self.conf, self.imgsz, self.device = conf, imgsz, device
        if model is None:
            os.environ.setdefault('YOLO_OFFLINE', '1')   # before importing Ultralytics
            from ultralytics import YOLO
            model = YOLO(weights)
        self.model = model
        self.last_inference_ms = 0.0

    def detect(self, frame_bgr: np.ndarray) -> list[dict]:
        """Run the model on one BGR frame; return spec v0.1 detection dicts (person only)."""
        t0 = time.perf_counter()
        result = self.model.predict(frame_bgr, conf=self.conf, classes=[PERSON_CLASS_ID],
                                    imgsz=self.imgsz, device=self.device, verbose=False)[0]
        self.last_inference_ms = (time.perf_counter() - t0) * 1e3
        out = []
        for k, box in enumerate(result.boxes):
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
            out.append({'det_id': k, 'class_name': 'person',
                        'confidence': round(float(box.conf), 4),
                        'bbox_xyxy': [round(v, 1) for v in (x1, y1, x2, y2)],
                        'bbox_center': [round((x1 + x2) / 2, 1), round((y1 + y2) / 2, 1)]})
        return out


def add_ranges(detections: list[dict], depth_m: np.ndarray | None) -> list[dict]:
    """Add "range_m" to each detection: median valid depth in the middle half of its box."""
    for d in detections:
        d['range_m'] = None
        if depth_m is None:
            continue
        x1, y1, x2, y2 = d['bbox_xyxy']
        sx, sy = depth_m.shape[1], depth_m.shape[0]
        w, h = x2 - x1, y2 - y1
        c0, c1 = int(max(0, x1 + w / 4)), int(min(sx, x2 - w / 4) + 1)
        r0, r1 = int(max(0, y1 + h / 4)), int(min(sy, y2 - h / 4) + 1)
        patch = depth_m[r0:r1, c0:c1]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size:
            d['range_m'] = round(float(np.median(valid)), 2)
    return detections


def frame_record(frame_id: int, source: str, width: int, height: int, inference_ms: float,
                 detections: list[dict]) -> dict:
    """One detection-output record (spec v0.1; video_time_s is None for a live camera)."""
    return {'frame_id': frame_id, 'source': source, 'video_time_s': None,
            'timestamp_utc': datetime.now(timezone.utc).isoformat(timespec='milliseconds')
            .replace('+00:00', 'Z'),
            'frame_width': width, 'frame_height': height,
            'inference_ms': round(inference_ms, 1), 'detections': detections}


def draw_detections(frame_bgr: np.ndarray, detections: list[dict]) -> np.ndarray:
    """Green boxes with "person 0.87" labels (and range when known) on a copy of the frame."""
    import cv2
    img = frame_bgr.copy()
    for d in detections:
        x1, y1, x2, y2 = (int(v) for v in d['bbox_xyxy'])
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{d['class_name']} {d['confidence']:.2f}"
        if d.get('range_m') is not None:
            label += f" {d['range_m']:.1f}m"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        top = max(y1 - th - 6, 0)
        cv2.rectangle(img, (x1, top), (x1 + tw + 4, top + th + 6), (0, 255, 0), -1)
        cv2.putText(img, label, (x1 + 2, top + th + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 0, 0), 1)
    return img

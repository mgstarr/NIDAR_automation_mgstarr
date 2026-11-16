"""Survivor detector node: colour camera -> YOLO11n -> detections for RViz and for tagging.

Subscribes: /camera/color/image_raw (sensor_msgs/Image, rgb8 or bgr8),
            /camera/depth/image_rect_raw (optional; 32FC1 metres or 16UC1 millimetres).
Publishes:  /detection/survivors (vision_msgs/Detection2DArray; class id "person", score,
            pixel box; header = the camera frame's header, so tagging can use TF at that time),
            /detection/image (sensor_msgs/Image, bgr8: the frame with boxes drawn),
            /detection/json (std_msgs/String: one detection-output record, spec v0.1 of the
            detection team + "range_m" per detection).
Parameters (config/detection.yaml): model_path (default: the handoff weights installed with this
package), confidence 0.403, image_size 416, device cpu, rate 3.0 Hz (frames in between are
skipped; detection is never in the flight-control loop).

Not here yet (later work): tracking / confirmation over frames, and tagging survivors on the map.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from airmouse_detection.detector import (SurvivorDetector, add_ranges, draw_detections,
                                         frame_record)

NODE_NAME = 'survivor_detector_node'


def image_to_bgr(msg: Image) -> np.ndarray | None:
    """sensor_msgs/Image (rgb8 / bgr8) -> BGR numpy array, without cv_bridge."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        return None
    arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
    return arr[:, :, ::-1].copy() if msg.encoding == 'rgb8' else arr.copy()


def depth_to_metres(msg: Image) -> np.ndarray | None:
    """Depth image -> float32 metres (32FC1 as is, 16UC1 millimetres / 1000)."""
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, -1)[:, :msg.width]
    if msg.encoding == '16UC1':
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, -1)[:, :msg.width] / 1000.0
    return None


class SurvivorDetectorNode(Node):
    """Runs the detector at a limited rate on the latest camera frame."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)
        default = str(Path(get_package_share_directory('airmouse_detection')) / 'models'
                      / 'survivor_yolo11n_demo.pt')
        model_path = self.declare_parameter('model_path', '').value or default
        conf = self.declare_parameter('confidence', 0.403).value
        imgsz = self.declare_parameter('image_size', 416).value
        device = self.declare_parameter('device', 'cpu').value
        self.period = 1.0 / self.declare_parameter('rate', 3.0).value
        self.detector = SurvivorDetector(model_path, conf, imgsz, device)
        self.depth: np.ndarray | None = None
        self.frame_id = 0
        self.last_run = -1e9
        self.total = 0
        self.create_subscription(Image, '/camera/color/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.on_depth,
                                 qos_profile_sensor_data)
        self.det_pub = self.create_publisher(Detection2DArray, '/detection/survivors', 10)
        self.img_pub = self.create_publisher(Image, '/detection/image', 2)
        self.json_pub = self.create_publisher(String, '/detection/json', 10)
        self.get_logger().info(f'ready: {Path(model_path).name}, conf {conf}, imgsz {imgsz}, '
                               f'{device}, {1 / self.period:.1f} Hz')

    def on_depth(self, msg: Image) -> None:
        """Keep the latest depth image (same resolution as colour in sim and on the D435i)."""
        self.depth = depth_to_metres(msg)

    def on_image(self, msg: Image) -> None:
        """Detect on this frame if the rate limit allows."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_run < self.period:
            return
        frame = image_to_bgr(msg)
        if frame is None:
            self.get_logger().warn(f'unsupported image encoding {msg.encoding}',
                                   throttle_duration_sec=10.0)
            return
        self.last_run = now
        dets = self.detector.detect(frame)
        depth = self.depth if self.depth is not None and self.depth.shape == frame.shape[:2] else None
        add_ranges(dets, depth)

        out = Detection2DArray()
        out.header = msg.header
        for d in dets:
            det = Detection2D()
            det.header = msg.header
            x1, y1, x2, y2 = d['bbox_xyxy']
            det.bbox.center.position.x, det.bbox.center.position.y = d['bbox_center']
            det.bbox.size_x, det.bbox.size_y = x2 - x1, y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id, hyp.hypothesis.score = 'person', d['confidence']
            if d['range_m'] is not None:
                hyp.pose.pose.position.z = d['range_m']     # camera optical frame: z forward
            det.results.append(hyp)
            det.id = str(d['det_id'])
            out.detections.append(det)
        self.det_pub.publish(out)

        record = frame_record(self.frame_id, 'camera', msg.width, msg.height,
                              self.detector.last_inference_ms, dets)
        self.json_pub.publish(String(data=json.dumps(record)))
        self.frame_id += 1

        annotated = draw_detections(frame, dets)
        img = Image()
        img.header = msg.header
        img.height, img.width = annotated.shape[:2]
        img.encoding, img.step = 'bgr8', annotated.shape[1] * 3
        img.data = annotated.tobytes()
        self.img_pub.publish(img)
        if dets:
            self.total += len(dets)
            best = max(dets, key=lambda d: d['confidence'])
            rng = f", {best['range_m']:.1f} m away" if best['range_m'] is not None else ''
            self.get_logger().info(
                f'SURVIVOR DETECTED: {len(dets)} in view, best confidence '
                f"{best['confidence']:.2f}{rng} ({self.detector.last_inference_ms:.0f} ms)",
                throttle_duration_sec=2.0)


def main(args: list[str] | None = None) -> None:
    """Run the detector node until shutdown.

    Args:
        args: Command-line arguments forwarded to rclpy.
    """
    rclpy.init(args=args)
    node = SurvivorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

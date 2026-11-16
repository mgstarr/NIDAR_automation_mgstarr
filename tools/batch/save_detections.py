#!/usr/bin/env python3
"""Save annotated detector frames that contain detections (test helper).

    python3 tools/batch/save_detections.py <out_dir> [--every 3.0] [--max 40]
Subscribes /detection/json and /detection/image; writes frame_<n>_<conf>.png (at most one per
`every` seconds) and counts.txt (frames, frames with detections) on exit.
"""

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

out = Path(sys.argv[1])
out.mkdir(parents=True, exist_ok=True)
every = float(sys.argv[sys.argv.index('--every') + 1]) if '--every' in sys.argv else 3.0
limit = int(sys.argv[sys.argv.index('--max') + 1]) if '--max' in sys.argv else 40
st = {'frames': 0, 'with': 0, 'last': 0.0, 'saved': 0, 'conf': 0.0, 'want': False}
rclpy.init()
node = Node('save_detections')


def on_json(msg: String) -> None:
    rec = json.loads(msg.data)
    st['frames'] += 1
    if rec['detections']:
        st['with'] += 1
        st['conf'] = max(d['confidence'] for d in rec['detections'])
        st['want'] = time.time() - st['last'] > every and st['saved'] < limit


def on_image(msg: Image) -> None:
    if not st['want']:
        return
    st['want'], st['last'] = False, time.time()
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
    cv2.imwrite(str(out / f"frame_{st['saved']:03d}_{st['conf']:.2f}.png"), img)
    st['saved'] += 1


node.create_subscription(String, '/detection/json', on_json, 10)
node.create_subscription(Image, '/detection/image', on_image, 2)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
(out / 'counts.txt').write_text(f"frames {st['frames']} with_detections {st['with']} saved {st['saved']}\n")

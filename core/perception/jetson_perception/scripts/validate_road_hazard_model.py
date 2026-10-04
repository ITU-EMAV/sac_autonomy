#!/usr/bin/env python3
"""Recorded RGB benchmark. Explicitly reports missing depth; never emits ROS commands."""
import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import time

import cv2
import numpy as np
import torch
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--bag', required=True, help='SQLite rosbag .db3')
    parser.add_argument('--output', required=True)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--device', default='0')
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.model, task='detect')
    names = {int(k): str(v) for k, v in model.names.items()}
    ids = [k for k, v in names.items() if v.lower().replace('-', '_').replace(' ', '_')
           in ('pothole', 'speed_bump')]
    if len(ids) != 2:
        raise RuntimeError(f'Unexpected classes: {names}')
    connection = sqlite3.connect(f'file:{Path(args.bag).resolve()}?mode=ro', uri=True)
    topics = connection.execute('SELECT id,name,type FROM topics').fetchall()
    rgb = next(t for t in topics if 'image_rect_color' in t[1] and t[2] == 'sensor_msgs/msg/Image')
    rows = connection.execute('SELECT id FROM messages WHERE topic_id=? ORDER BY timestamp',
                              (rgb[0],)).fetchall()
    selected = np.linspace(0, len(rows)-1, min(args.frames, len(rows)), dtype=int)
    bridge = CvBridge()
    latency, detections, samples = [], [], []
    for index, offset in enumerate(selected):
        raw = connection.execute('SELECT data FROM messages WHERE id=?', (rows[offset][0],)).fetchone()[0]
        msg = deserialize_message(raw, Image)
        image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        if index == 0:
            for _ in range(3):
                model.predict(image, device=args.device, imgsz=640, conf=.35, classes=ids, verbose=False)
        if args.device != 'cpu':
            torch.cuda.synchronize()
        start = time.perf_counter()
        result = model.predict(image, device=args.device, imgsz=640, conf=.35,
                               classes=ids, verbose=False)[0]
        if args.device != 'cpu':
            torch.cuda.synchronize()
        elapsed = (time.perf_counter()-start)*1000
        latency.append(elapsed)
        frame_detections = []
        for box in result.boxes:
            item = {'class_id': int(box.cls[0]), 'class_name': names[int(box.cls[0])],
                    'confidence': float(box.conf[0]), 'bbox_xyxy': box.xyxy[0].tolist(),
                    'stamp_sec': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                    'distance_m': None, 'path_intersection': None}
            detections.append(item)
            frame_detections.append(item)
        if frame_detections and len(samples) < 8:
            file = output/f'detection_{index:04d}.jpg'
            cv2.imwrite(str(file), result.plot())
            samples.append(str(file))
        if index % 20 == 0:
            print(f'{index+1}/{len(selected)} frames, {elapsed:.1f} ms, {len(detections)} hazards', flush=True)
    summary = {'model': str(Path(args.model).resolve()),
               'sha256': hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
               'bag': args.bag, 'topics': topics, 'classes': names, 'hazard_class_ids': ids,
               'sampled_frames': len(selected), 'warmup_frames': 3,
               'latency_ms': {'mean': float(np.mean(latency)), 'p50': float(np.median(latency)),
                              'p95': float(np.percentile(latency, 95)), 'max': max(latency)},
               'counts': {names[k]: sum(d['class_id']==k for d in detections) for k in ids},
               'validation_scope': 'Recorded RGB inference only; no depth/path/vehicle actuation',
               'depth_recorded': any('depth' in t[1] and t[2]=='sensor_msgs/msg/Image' for t in topics),
               'detections': detections, 'sample_images': samples}
    (output/'validation.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps({k: v for k, v in summary.items() if k not in ('detections','topics')}, indent=2))


if __name__ == '__main__':
    main()

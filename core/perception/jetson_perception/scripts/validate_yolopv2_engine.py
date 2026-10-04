#!/usr/bin/env python3
"""Compare YOLOPv2 TorchScript and TensorRT road masks on recorded RGB frames."""
import argparse
import importlib
import json
from pathlib import Path
import sqlite3
import sys
import time
import types

import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
import torch


def mask_iou(a, b):
    a, b = a > 0, b > 0
    union = np.count_nonzero(a | b)
    return float(np.count_nonzero(a & b) / union) if union else 1.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--engine', type=Path, required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=30)
    args = parser.parse_args()
    try:
        importlib.import_module('torchvision')
    except ModuleNotFoundError:
        sys.modules['torchvision'] = types.ModuleType('torchvision')
    repo = Path(__file__).resolve().parents[1] / 'third_party/YOLOPv2'
    sys.path.insert(0, str(repo))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from utils.utils import driving_area_mask, lane_line_mask, letterbox
    from jetson_perception.tensorrt_road import TensorRTRoadMasks

    device = torch.device('cuda:0')
    scripted = torch.jit.load(str(args.weights), map_location=device).eval().half()
    engine = TensorRTRoadMasks(args.engine, device)
    connection = sqlite3.connect(f'file:{args.bag.resolve()}?mode=ro', uri=True)
    topic = connection.execute(
        "SELECT id FROM topics WHERE name='/zed/zed_node/left/image_rect_color'").fetchone()[0]
    rows = connection.execute(
        'SELECT id FROM messages WHERE topic_id=? ORDER BY timestamp', (topic,)).fetchall()
    if not rows:
        raise RuntimeError('No RGB images in bag')
    selected = np.linspace(0, len(rows)-1, min(args.frames, len(rows)), dtype=int)
    bridge = CvBridge()
    results = []
    for index, offset in enumerate(selected):
        raw = connection.execute('SELECT data FROM messages WHERE id=?',
                                 (rows[offset][0],)).fetchone()[0]
        msg = deserialize_message(raw, Image)
        original = bridge.imgmsg_to_cv2(msg, 'bgr8')
        resized = cv2.resize(original, (1280, 720), interpolation=cv2.INTER_LINEAR)
        network, _, _ = letterbox(resized, 640, stride=32)
        network = np.ascontiguousarray(network[:, :, ::-1].transpose(2, 0, 1))
        input32 = torch.from_numpy(network).to(device).float().unsqueeze(0) / 255.0
        input16 = input32.half()
        if index == 0:
            with torch.inference_mode():
                for _ in range(3):
                    scripted(input16)
                    engine(input32)
            torch.cuda.synchronize()
        torch.cuda.synchronize()
        start = time.perf_counter()
        with torch.inference_mode():
            [_pred, _anchor_grid], road_pt, lane_pt = scripted(input16)
        torch.cuda.synchronize()
        pt_ms = (time.perf_counter()-start)*1000.0
        start = time.perf_counter()
        road_trt, lane_trt = engine(input32)
        torch.cuda.synchronize()
        trt_ms = (time.perf_counter()-start)*1000.0
        road_pt_mask = driving_area_mask(road_pt)
        lane_pt_mask = lane_line_mask(lane_pt)
        road_trt_mask = driving_area_mask(road_trt)
        lane_trt_mask = lane_line_mask(lane_trt)
        results.append({
            'frame': int(offset), 'pt_ms': pt_ms, 'trt_ms': trt_ms,
            'road_iou': mask_iou(road_pt_mask, road_trt_mask),
            'lane_iou': mask_iou(lane_pt_mask, lane_trt_mask),
        })
        if index % 10 == 0:
            print(f'{index+1}/{len(selected)}: pt={pt_ms:.1f} ms trt={trt_ms:.1f} ms', flush=True)
    summary = {
        'frames': len(results),
        'pt_ms_mean': float(np.mean([row['pt_ms'] for row in results])),
        'trt_ms_mean': float(np.mean([row['trt_ms'] for row in results])),
        'road_iou_mean': float(np.mean([row['road_iou'] for row in results])),
        'road_iou_min': min(row['road_iou'] for row in results),
        'lane_iou_mean': float(np.mean([row['lane_iou'] for row in results])),
        'lane_iou_min': min(row['lane_iou'] for row in results),
        'samples': results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2))
    print(json.dumps({key: value for key, value in summary.items() if key != 'samples'}, indent=2))


if __name__ == '__main__':
    main()

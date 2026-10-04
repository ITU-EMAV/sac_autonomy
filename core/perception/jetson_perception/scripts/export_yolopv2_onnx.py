#!/usr/bin/env python3
"""Export only the YOLOPv2 road and lane logits used by the ROS road node."""
import argparse
from pathlib import Path

import onnx
import torch


class RoadMasks(torch.nn.Module):
    def __init__(self, scripted):
        super().__init__()
        self.scripted = scripted

    def forward(self, image):
        _detections, drivable, lane = self.scripted(image)
        return drivable, lane


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    model = torch.jit.script(
        RoadMasks(torch.jit.load(str(args.weights), map_location='cpu').eval()).eval())
    example = torch.zeros((1, 3, 384, 640), dtype=torch.float32)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with torch.inference_mode():
        torch.onnx.export(
            model, example, str(args.output), opset_version=17,
            input_names=['images'], output_names=['drivable_logits', 'lane_logits'],
            do_constant_folding=True)
    graph = onnx.load(str(args.output))
    onnx.checker.check_model(graph)
    print('ONNX exported:', args.output, args.output.stat().st_size,
          'bytes; outputs:', [output.name for output in graph.graph.output], flush=True)


if __name__ == '__main__':
    main()

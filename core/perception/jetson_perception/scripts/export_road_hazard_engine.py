#!/usr/bin/env python3
"""Build the FP16 engine on the target Jetson; never train or calibrate INT8."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform

os.environ['YOLO_AUTOINSTALL'] = 'false'
import torch
import tensorrt
from ultralytics import YOLO, __version__ as ultralytics_version


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    hardware = Path('/proc/device-tree/model').read_text().rstrip('\x00')
    if 'Orin' not in hardware or platform.machine() != 'aarch64' or not torch.cuda.is_available():
        raise RuntimeError('Build this engine on the target Jetson Orin with CUDA available')
    source, target = Path(args.model).resolve(), Path(args.output).resolve()
    if target.suffix != '.engine' or source.suffix != '.pt':
        raise ValueError('Expected .pt input and .engine output')
    model = YOLO(str(source), task='detect')
    names = {str(v).lower().replace('-', '_').replace(' ', '_') for v in model.names.values()}
    if not {'pothole', 'speed_bump'}.issubset(names):
        raise ValueError(f'Wrong model classes: {model.names}')
    engine = Path(model.export(format='engine', half=True, device=0, imgsz=640,
                               batch=1, dynamic=False, workspace=2.0, simplify=False))
    if engine.resolve() != target:
        engine.replace(target)
    metadata = {'source': str(source), 'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                'engine_sha256': hashlib.sha256(target.read_bytes()).hexdigest(),
                'device': hardware, 'gpu': torch.cuda.get_device_name(0),
                'torch': torch.__version__, 'tensorrt': tensorrt.__version__,
                'ultralytics': ultralytics_version, 'classes': model.names,
                'precision': 'FP16', 'input': [1, 3, 640, 640],
                'note': 'Target-local engine. Rebuild after incompatible JetPack/TensorRT/GPU changes.'}
    target.with_suffix('.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()

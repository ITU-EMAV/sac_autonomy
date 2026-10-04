"""TensorRT 10 road-mask runtime for a fixed 1x3x384x640 YOLOPv2 engine."""
from pathlib import Path

import tensorrt as trt
import torch


TORCH_DTYPES = {
    trt.float32: torch.float32,
    trt.float16: torch.float16,
}


class TensorRTRoadMasks:
    def __init__(self, path, device):
        if device.type != 'cuda':
            raise ValueError('TensorRT road masks require CUDA')
        self.device = device
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(Path(path).read_bytes())
        if self.engine is None:
            raise RuntimeError(f'Unable to load TensorRT road engine: {path}')
        self.context = self.engine.create_execution_context()
        expected = {'images', 'drivable_logits', 'lane_logits'}
        names = {self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)}
        if names != expected:
            raise ValueError(f'Unexpected road engine tensors: {names}')
        if tuple(self.engine.get_tensor_shape('images')) != (1, 3, 384, 640):
            raise ValueError('Road engine input must be fixed 1x3x384x640')
        self.input_dtype = TORCH_DTYPES.get(self.engine.get_tensor_dtype('images'))
        if self.input_dtype is None:
            raise ValueError('Unsupported road engine input dtype')
        self.outputs = {}
        for name in ('drivable_logits', 'lane_logits'):
            dtype = TORCH_DTYPES.get(self.engine.get_tensor_dtype(name))
            if dtype is None:
                raise ValueError(f'Unsupported road engine output dtype: {name}')
            shape = tuple(self.engine.get_tensor_shape(name))
            if any(value <= 0 for value in shape):
                raise ValueError(f'Road engine output shape is not fixed: {name}: {shape}')
            self.outputs[name] = torch.empty(shape, dtype=dtype, device=device)
            self.context.set_tensor_address(name, self.outputs[name].data_ptr())

    def __call__(self, image):
        if (tuple(image.shape) != (1, 3, 384, 640) or image.dtype != self.input_dtype or
                image.device != self.device or not image.is_contiguous()):
            raise ValueError('Road engine expects contiguous 1x3x384x640 CUDA input')
        self.context.set_tensor_address('images', image.data_ptr())
        stream = torch.cuda.current_stream(self.device)
        if not self.context.execute_async_v3(stream_handle=stream.cuda_stream):
            raise RuntimeError('TensorRT road inference failed')
        return self.outputs['drivable_logits'], self.outputs['lane_logits']

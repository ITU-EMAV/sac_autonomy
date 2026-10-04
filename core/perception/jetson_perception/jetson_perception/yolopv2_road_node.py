import importlib
import fcntl
import sys
import time
import types
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from sac_interfaces.msg import PipelineTiming
import torch
from .asset_paths import asset_path


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class Yolopv2RoadNode(Node):
    def __init__(self):
        super().__init__("yolopv2_road_node")
        self.declare_parameter("image_topic", "/zed/zed_node/left/image_rect_color")
        self.declare_parameter(
            "weights", "models/yolopv2.pt"
        )
        self.declare_parameter(
            "yolopv2_repo",
            "third_party/YOLOPv2",
        )
        self.declare_parameter("img_size", 640)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("conf_threshold", 0.30)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_fps", 15.0)
        self.declare_parameter("publish_visualization", False)
        self.declare_parameter("serialize_gpu", False)

        self.image_topic = self.get_parameter("image_topic").value
        self.img_size = int(self.get_parameter("img_size").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.publish_visualization = bool(self.get_parameter("publish_visualization").value)
        self.serialize_gpu = bool(self.get_parameter("serialize_gpu").value)
        self._gpu_lock_file = open("/tmp/sac_gpu_inference.lock", "w") if self.serialize_gpu else None
        requested_device = str(self.get_parameter("device").value)
        if requested_device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("CUDA was requested for YOLOPv2, but torch.cuda.is_available() is false")
        self.device = torch.device(requested_device)

        repo = Path(asset_path(self.get_parameter("yolopv2_repo").value))
        weights = Path(asset_path(self.get_parameter("weights").value))
        self.use_tensorrt = weights.suffix == '.engine'
        self.half = self.device.type == "cuda" and not self.use_tensorrt
        if not repo.is_dir():
            raise FileNotFoundError(f"YOLOPv2 repository not found: {repo}")
        if not weights.is_file() or weights.stat().st_size == 0:
            raise FileNotFoundError(f"YOLOPv2 weights not found or empty: {weights}")

        # Upstream utils imports torchvision only for object-detection NMS. This node
        # uses the official segmentation helpers, so allow that module to load on the
        # Jetson image where torchvision is intentionally not installed.
        try:
            importlib.import_module("torchvision")
        except ModuleNotFoundError:
            sys.modules["torchvision"] = types.ModuleType("torchvision")
        sys.path.insert(0, str(repo))
        from utils.utils import driving_area_mask, lane_line_mask, letterbox

        self.driving_area_mask = driving_area_mask
        self.lane_line_mask = lane_line_mask
        self.letterbox = letterbox

        self.get_logger().info(f"Loading YOLOPv2 road masks once from {weights} on {self.device}")
        if self.use_tensorrt:
            if self.img_size != 640:
                raise ValueError('This TensorRT road engine requires img_size=640')
            from .tensorrt_road import TensorRTRoadMasks
            self.model = TensorRTRoadMasks(weights, self.device)
        else:
            self.model = torch.jit.load(str(weights), map_location=self.device).to(self.device)
            if self.half:
                self.model.half()
            self.model.eval()
        with torch.inference_mode():
            warmup = torch.zeros((1, 3, 384, self.img_size), device=self.device,
                                 dtype=torch.float16 if self.half else torch.float32)
            self.model(warmup)
            if self.device.type == 'cuda':
                torch.cuda.synchronize()

        self.bridge = CvBridge()
        self.drivable_pub = self.create_publisher(
            Image, "/perception/road/drivable_mask", SENSOR_QOS
        )
        self.lane_pub = self.create_publisher(Image, "/perception/road/lane_mask", SENSOR_QOS)
        self.overlay_pub = self.create_publisher(Image, "/perception/road/overlay", SENSOR_QOS)
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, SENSOR_QOS
        )
        self.timing_pub = self.create_publisher(PipelineTiming, "/perception/road/timing", 1)
        self._latest_lock = threading.Lock()
        self._latest_msg = None
        self._work_event = threading.Event()
        self._stop_event = threading.Event()
        self.received_count = 0
        self.processed_count = 0
        self.dropped_count = 0
        self._worker = threading.Thread(target=self._worker_loop, name="road_latest_frame", daemon=False)
        self._worker.start()
        self.last_start = 0.0
        self.frame_count = 0
        self.stats = np.zeros(4, dtype=np.float64)
        self.stats_start = time.monotonic()
        self.get_logger().info(f"YOLOPv2 road node ready; subscribed to {self.image_topic}")

    def image_callback(self, msg):
        self.received_count += 1
        with self._latest_lock:
            if self._latest_msg is not None:
                self.dropped_count += 1
            self._latest_msg = msg
        self._work_event.set()

    def _worker_loop(self):
        while not self._stop_event.is_set():
            self._work_event.wait(timeout=0.1)
            self._work_event.clear()
            with self._latest_lock:
                msg = self._latest_msg
                self._latest_msg = None
            if msg is not None:
                self._process_image(msg)

    def stop_worker(self):
        self._stop_event.set()
        self._work_event.set()
        self._worker.join(timeout=3.0)

    def _process_image(self, msg):
        now = time.monotonic()
        if self.max_fps > 0.0 and now - self.last_start < 1.0 / self.max_fps:
            self.dropped_count += 1
            return
        self.last_start = now
        total_start = time.perf_counter()

        try:
            original = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            original_h, original_w = original.shape[:2]

            prep_start = time.perf_counter()
            # The official demo normalizes every source to 1280x720, then applies its
            # stride-aware letterbox. ZED's 16:9 input maps without geometric skew.
            model_canvas = cv2.resize(original, (1280, 720), interpolation=cv2.INTER_LINEAR)
            network_image, _, _ = self.letterbox(
                model_canvas, self.img_size, stride=32
            )
            network_image = network_image[:, :, ::-1].transpose(2, 0, 1)
            network_image = np.ascontiguousarray(network_image)
            prep_end = time.perf_counter()

            if self._gpu_lock_file is not None:
                fcntl.flock(self._gpu_lock_file, fcntl.LOCK_EX)
            try:
                tensor = torch.from_numpy(network_image).to(self.device)
                tensor = tensor.half() if self.half else tensor.float()
                tensor = tensor.unsqueeze(0) / 255.0
                with torch.inference_mode():
                    if self.use_tensorrt:
                        seg, lane = self.model(tensor)
                    else:
                        [_pred, _anchor_grid], seg, lane = self.model(tensor)
                    if self.device.type == 'cuda':
                        torch.cuda.synchronize()
            finally:
                if self._gpu_lock_file is not None:
                    fcntl.flock(self._gpu_lock_file, fcntl.LOCK_UN)
            infer_end = time.perf_counter()

            drivable = self.driving_area_mask(seg)
            lane_mask = self.lane_line_mask(lane)
            drivable = cv2.resize(
                drivable.astype(np.uint8), (original_w, original_h), interpolation=cv2.INTER_NEAREST
            )
            lane_mask = cv2.resize(
                lane_mask.astype(np.uint8), (original_w, original_h), interpolation=cv2.INTER_NEAREST
            )
            drivable_mono = (drivable > 0).astype(np.uint8) * 255
            lane_mono = (lane_mask > 0).astype(np.uint8) * 255

            overlay = None
            if self.publish_visualization:
                overlay = original.copy()
                color = np.zeros_like(overlay)
                color[drivable_mono > 0] = (0, 170, 0)
                color[lane_mono > 0] = (0, 0, 255)
                active = (drivable_mono > 0) | (lane_mono > 0)
                overlay[active] = cv2.addWeighted(overlay, 0.55, color, 0.45, 0)[active]
            post_end = time.perf_counter()

            if self._stop_event.is_set() or not rclpy.ok():
                return

            self._publish_image(self.drivable_pub, drivable_mono, "mono8", msg)
            self._publish_image(self.lane_pub, lane_mono, "mono8", msg)
            if overlay is not None:
                self._publish_image(self.overlay_pub, overlay, "bgr8", msg)

            total_end = time.perf_counter()
            self.stats += np.array([
                prep_end - prep_start,
                infer_end - prep_end,
                post_end - infer_end,
                total_end - total_start,
            ])
            self.frame_count += 1
            self.processed_count += 1
            metric = PipelineTiming()
            metric.stamp = self.get_clock().now().to_msg()
            metric.component = "yolopv2_road"
            metric.sequence = self.processed_count
            metric.execution_time_ms = (total_end - total_start) * 1000.0
            metric.source_age_ms = (self.get_clock().now().nanoseconds -
                                    rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds) / 1e6
            metric.queue_delay_ms = metric.source_age_ms - metric.execution_time_ms
            metric.received_count = self.received_count
            metric.processed_count = self.processed_count
            metric.dropped_count = self.dropped_count
            self.timing_pub.publish(metric)
            if self.frame_count % 30 == 0:
                elapsed = max(time.monotonic() - self.stats_start, 1e-9)
                avg_ms = self.stats / 30.0 * 1000.0
                self.get_logger().info(
                    "stats fps=%.2f pre=%.1fms infer=%.1fms post=%.1fms total=%.1fms"
                    % (30.0 / elapsed, avg_ms[0], avg_ms[1], avg_ms[2], avg_ms[3])
                )
                self.stats[:] = 0.0
                self.stats_start = time.monotonic()
        except Exception as exc:
            if not self._stop_event.is_set() and rclpy.ok():
                self.get_logger().error(
                    f"YOLOPv2 frame failed: {exc}", throttle_duration_sec=5.0)

    def _publish_image(self, publisher, image, encoding, source):
        output = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        output.header = source.header
        publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Yolopv2RoadNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

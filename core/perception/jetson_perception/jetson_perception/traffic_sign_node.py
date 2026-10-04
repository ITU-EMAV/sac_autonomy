import json
import fcntl
import time
import threading
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException
from jetson_perception.road_hazard_node import project_box
from std_msgs.msg import String
from sac_interfaces.msg import (CameraDetection, CameraDetectionArray, DetectedObject,
                                DetectedObjectArray, PipelineTiming)
import torch
from .asset_paths import asset_path


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class TrafficSignNode(Node):
    def __init__(self):
        super().__init__("traffic_sign_node")
        self.declare_parameter("image_topic", "/zed/zed_node/left/image_rect_color")
        self.declare_parameter(
            "model_path",
            "models/traffic_sign_22cls.engine",
        )
        self.declare_parameter(
            "object_model_path",
            "models/yolov8n.engine",
        )
        self.declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter("camera_info_topic", "/zed/zed_node/left/camera_info")
        self.declare_parameter("depth_sync_slop", 0.04)
        self.declare_parameter("img_size", 640)
        self.declare_parameter("object_img_size", 512)
        self.declare_parameter("sign_confidence", 0.45)
        self.declare_parameter("object_confidence", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("object_classes", [0, 2, 3, 5, 7])
        self.declare_parameter("device", "0")
        self.declare_parameter("max_fps", 15.0)
        self.declare_parameter("publish_visualization", False)
        self.declare_parameter("serialize_gpu", False)

        if not torch.cuda.is_available():
            raise RuntimeError("CUDA inference requested, but CUDA is unavailable")
        from ultralytics import YOLO

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.sign_model_path = asset_path(self.get_parameter("model_path").value)
        self.object_model_path = asset_path(self.get_parameter("object_model_path").value)
        self.sign_img_size = int(self.get_parameter("img_size").value)
        self.object_img_size = int(self.get_parameter("object_img_size").value)
        self.sign_confidence = float(self.get_parameter("sign_confidence").value)
        self.object_confidence = float(self.get_parameter("object_confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.object_classes = [
            int(value) for value in self.get_parameter("object_classes").value
        ]
        self.device = str(self.get_parameter("device").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.publish_visualization = bool(self.get_parameter("publish_visualization").value)
        self.serialize_gpu = bool(self.get_parameter("serialize_gpu").value)
        self._gpu_lock_file = open("/tmp/sac_gpu_inference.lock", "w") if self.serialize_gpu else None

        self.get_logger().info(f"Loading sign model once from {self.sign_model_path}")
        self.sign_model = YOLO(self.sign_model_path)
        self.get_logger().info(f"Loading object model once from {self.object_model_path}")
        self.object_model = YOLO(self.object_model_path)

        self.bridge = CvBridge()
        self._depth_lock = threading.Lock()
        self._depth_frames = deque(maxlen=20)
        self._camera_info = None
        self.depth_subscription = self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self.depth_callback, SENSOR_QOS)
        self.info_subscription = self.create_subscription(
            CameraInfo, self.get_parameter("camera_info_topic").value,
            self.info_callback, SENSOR_QOS)
        self.overlay_pub = self.create_publisher(Image, "/perception/signs/overlay", SENSOR_QOS)
        self.json_pub = self.create_publisher(
            String, "/perception/signs/detections_json", QoSProfile(depth=1)
        )
        self.typed_pub = self.create_publisher(
            CameraDetectionArray, "/yolo_detections", QoSProfile(depth=1)
        )
        self.object_pub = self.create_publisher(
            DetectedObjectArray, '/perception/dynamic_objects', QoSProfile(depth=1))
        self.object_timing_pub = self.create_publisher(
            PipelineTiming, '/perception/dynamic_objects/timing', 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, SENSOR_QOS
        )
        self.timing_pub = self.create_publisher(PipelineTiming, "/perception/signs/timing", 1)
        self._latest_lock = threading.Lock()
        self._latest_msg = None
        self._work_event = threading.Event()
        self._stop_event = threading.Event()
        self.received_count = 0
        self.processed_count = 0
        self.dropped_count = 0
        self._worker = threading.Thread(target=self._worker_loop, name="sign_latest_frame", daemon=False)
        self.last_start = 0.0
        self.frame_count = 0
        self.stats = np.zeros(2, dtype=np.float64)
        self.stats_start = time.monotonic()
        self.run_sign_next = True
        self._worker.start()
        self.get_logger().info(
            "Combined detector ready; sign and road-object models alternate frames"
        )

    def depth_callback(self, msg):
        with self._depth_lock:
            self._depth_frames.append(msg)

    def info_callback(self, msg):
        with self._depth_lock:
            self._camera_info = msg

    def metric_depth(self, rgb):
        with self._depth_lock:
            info = self._camera_info
            frames = list(self._depth_frames)
        if info is None or not frames:
            return None
        stamp = rclpy.time.Time.from_msg(rgb.header.stamp).nanoseconds
        depth = min(frames, key=lambda m: abs(
            rclpy.time.Time.from_msg(m.header.stamp).nanoseconds-stamp))
        delta = abs(rclpy.time.Time.from_msg(depth.header.stamp).nanoseconds-stamp)/1e9
        if (delta > self.get_parameter("depth_sync_slop").value or
                (rgb.width, rgb.height) != (depth.width, depth.height) or
                (rgb.width, rgb.height) != (info.width, info.height) or
                not rgb.header.frame_id or rgb.header.frame_id != info.header.frame_id or
                rgb.header.frame_id != depth.header.frame_id or
                depth.encoding not in ('32FC1', '16UC1')):
            return None
        return (self.bridge.imgmsg_to_cv2(depth, 'passthrough'),
                (info.p[0], info.p[5], info.p[2], info.p[6]),
                .001 if depth.encoding == '16UC1' else 1.0)

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
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            source = "traffic_sign" if self.run_sign_next else "road_object"
            model = self.sign_model if self.run_sign_next else self.object_model
            size = self.sign_img_size if self.run_sign_next else self.object_img_size
            classes = None if self.run_sign_next else self.object_classes
            confidence = (
                self.sign_confidence if self.run_sign_next else self.object_confidence
            )
            self.run_sign_next = not self.run_sign_next

            if self._gpu_lock_file is not None:
                fcntl.flock(self._gpu_lock_file, fcntl.LOCK_EX)
            try:
                result = model.predict(
                    source=image,
                    imgsz=size,
                    conf=confidence,
                    iou=self.iou,
                    device=self.device,
                    classes=classes,
                    verbose=False,
                )[0]
                torch.cuda.synchronize()
            finally:
                if self._gpu_lock_file is not None:
                    fcntl.flock(self._gpu_lock_file, fcntl.LOCK_UN)
            infer_end = time.perf_counter()
            detections = self._detections(result, source)

            if self._stop_event.is_set() or not rclpy.ok():
                return

            overlay = image.copy() if self.publish_visualization else None
            for detection in detections if overlay is not None else []:
                x1, y1, x2, y2 = (
                    int(round(value)) for value in detection["bbox_xyxy"]
                )
                color = (0, 255, 255) if detection["source"] == "traffic_sign" else (255, 180, 0)
                cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
                cv2.putText(
                    overlay,
                    f"{detection['class_name']} {detection['confidence']:.2f}",
                    (x1, max(20, y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )

            if overlay is not None:
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                overlay_msg.header = msg.header
                self.overlay_pub.publish(overlay_msg)
            payload = {
                "source": source,
                "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                "frame_id": msg.header.frame_id,
                "detections": detections,
            }
            self.json_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
            typed = CameraDetectionArray()
            typed.header = msg.header
            typed.sort_order = "unsorted"
            metric_depth = self.metric_depth(msg)
            for detection in detections if source == "traffic_sign" else []:
                item = CameraDetection()
                class_id = int(detection["class_id"])
                item.label = class_id if 0 <= class_id <= CameraDetection.UNKNOWN else CameraDetection.UNKNOWN
                item.confidence = float(detection["confidence"])
                item.bbox = [max(0, min(65535, int(round(value))))
                             for value in detection["bbox_xyxy"]]
                item.distance = -1.0
                if metric_depth is not None:
                    depth, intrinsic, scale = metric_depth
                    try:
                        point, _ = project_box(depth, detection["bbox_xyxy"], intrinsic, scale)
                        item.distance = float(np.linalg.norm(point))
                    except ValueError:
                        pass  # Unknown depth is explicitly represented, never guessed.
                typed.detections.append(item)
            if source == "traffic_sign":
                self.typed_pub.publish(typed)
            else:
                metric_start = time.perf_counter()
                objects = DetectedObjectArray()
                objects.header = msg.header
                names = {0: DetectedObject.PERSON, 2: DetectedObject.CAR,
                         3: DetectedObject.MOTORCYCLE, 5: DetectedObject.BUS,
                         7: DetectedObject.TRUCK}
                for detection in detections:
                    kind = names.get(int(detection['class_id']))
                    if kind is None:
                        continue
                    item = DetectedObject()
                    item.kind = kind
                    item.confidence = float(detection['confidence'])
                    item.bbox_xyxy = [float(v) for v in detection['bbox_xyxy']]
                    item.source_header = msg.header
                    item.distance_m = -1.0
                    if metric_depth is not None:
                        depth, intrinsic, scale = metric_depth
                        try:
                            point, _ = project_box(depth, detection['bbox_xyxy'],
                                                   intrinsic, scale, center_fraction=.65)
                            item.optical_position.x, item.optical_position.y, item.optical_position.z = point
                            item.position_valid = True
                            item.distance_m = float(np.linalg.norm(point))
                            try:
                                tf = self.tf_buffer.lookup_transform(
                                    'base_link', msg.header.frame_id,
                                    rclpy.time.Time.from_msg(msg.header.stamp))
                                q = tf.transform.rotation
                                xyz = np.array(point)
                                qv = np.array([q.x, q.y, q.z])
                                xyz = xyz + 2.0 * np.cross(qv, np.cross(qv, xyz) + q.w * xyz)
                                xyz += np.array([tf.transform.translation.x,
                                                 tf.transform.translation.y,
                                                 tf.transform.translation.z])
                                item.base_position.x, item.base_position.y, item.base_position.z = xyz.tolist()
                                item.base_position_valid = True
                            except TransformException:
                                pass
                        except ValueError:
                            pass
                    objects.objects.append(item)
                self.object_pub.publish(objects)
                object_timing = PipelineTiming()
                object_timing.stamp = self.get_clock().now().to_msg()
                object_timing.component = 'dynamic_object_metric'
                object_timing.sequence = self.processed_count + 1
                object_timing.execution_time_ms = (time.perf_counter()-metric_start)*1000.0
                object_timing.source_age_ms = (self.get_clock().now().nanoseconds-
                    rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds)/1e6
                object_timing.received_count = self.received_count
                object_timing.processed_count = self.processed_count + 1
                object_timing.dropped_count = self.dropped_count
                self.object_timing_pub.publish(object_timing)

            total_end = time.perf_counter()
            self.stats += np.array([infer_end - total_start, total_end - total_start])
            self.frame_count += 1
            self.processed_count += 1
            metric = PipelineTiming()
            metric.stamp = self.get_clock().now().to_msg()
            metric.component = "traffic_sign"
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
                    "stats fps=%.2f infer=%.1fms total=%.1fms detections=%d"
                    % (30.0 / elapsed, avg_ms[0], avg_ms[1], len(detections))
                )
                self.stats[:] = 0.0
                self.stats_start = time.monotonic()
        except Exception as exc:
            if not self._stop_event.is_set() and rclpy.ok():
                self.get_logger().error(
                    f"Combined detection frame failed: {exc}", throttle_duration_sec=5.0
                )

    @staticmethod
    def _detections(result, source):
        detections = []
        if result.boxes is None:
            return detections
        for box in result.boxes:
            class_id = int(box.cls[0].item())
            detections.append({
                "source": source,
                "class_id": class_id,
                "class_name": str(result.names[class_id]),
                "confidence": float(box.conf[0].item()),
                "bbox_xyxy": [float(value) for value in box.xyxy[0].tolist()],
            })
        return detections


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrafficSignNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_worker()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

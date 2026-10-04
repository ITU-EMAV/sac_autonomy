"""Optional hazard model; synchronized registered RGB/depth, no actuator output."""
import fcntl
import math
import os
import time

import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from sac_interfaces.msg import PipelineTiming, RoadHazard, RoadHazardArray


def project_box(depth, box, intrinsic, scale=1.0, center_fraction=0.5):
    """Robust central ROI depth; return optical point and conservative box radius."""
    height, width = depth.shape
    x1, y1, x2, y2 = map(float, box)
    if not all(math.isfinite(v) for v in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
        raise ValueError('invalid bounding box')
    x1, x2 = max(0.0, x1), min(float(width-1), x2)
    y1, y2 = max(0.0, y1), min(float(height-1), y2)
    if x2 <= x1 or y2 <= y1 or not 0.2 <= center_fraction <= 0.8:
        raise ValueError('bounding box outside image or invalid ROI center')
    u, v = (x1 + x2) / 2, y1 + center_fraction*(y2-y1)
    dx, dy = max(2, (x2 - x1) * .2), max(2, (y2 - y1) * .2)
    roi = depth[max(0, int(v-dy)):min(height, int(v+dy)+1),
                max(0, int(u-dx)):min(width, int(u+dx)+1)].astype(float) * scale
    good = roi[np.isfinite(roi) & (roi > .2) & (roi < 40.0)]
    if roi.size == 0 or good.size < max(5, roi.size * .5):
        raise ValueError('insufficient valid depth')
    z = float(np.median(good))
    if np.percentile(good, 90) - np.percentile(good, 10) > max(.3, .15*z):
        raise ValueError('ambiguous depth ROI')
    fx, fy, cx, cy = intrinsic
    if not all(math.isfinite(a) for a in intrinsic) or min(fx, fy) <= 0:
        raise ValueError('invalid calibration')
    return ((u-cx)*z/fx, (v-cy)*z/fy, z), max(.15, (x2-x1)*z/(2*fx))


class RoadHazardNode(Node):
    def __init__(self):
        super().__init__('road_hazard_node')
        defaults = {
            'model_path': '', 'image_topic': '/zed/zed_node/left/image_rect_color',
            'depth_topic': '/zed/zed_node/depth/depth_registered',
            'camera_info_topic': '/zed/zed_node/left/camera_info',
            'confidence': .35, 'img_size': 640, 'device': '0', 'max_fps': 5.0,
            'max_source_age': .5, 'future_stamp_tolerance': .1,
            'sync_slop': .04, 'serialize_gpu': True,
            'publish_visualization': False, 'rgb_only_preview': False,
            'auto_depth_preview': False,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        if not os.path.isfile(self.p('model_path')):
            raise RuntimeError('Set model_path to a local pothole/speed_bump .pt or .engine file')
        from ultralytics import YOLO
        self.model = YOLO(self.p('model_path'), task='detect')
        self.names = {int(k): str(v).lower().replace('-', '_').replace(' ', '_')
                      for k, v in self.model.names.items()}
        if not {'pothole', 'speed_bump'}.issubset(set(self.names.values())):
            raise ValueError('model must contain named pothole and speed_bump classes')
        self.hazard_class_ids = [k for k, v in self.names.items() if v in ('pothole', 'speed_bump')]
        self.get_logger().info(f'Loaded classes: {self.model.names}; hazard IDs: {self.hazard_class_ids}')
        self.sequence = 0
        self.received = 0
        self.timing_pub = self.create_publisher(PipelineTiming, '/perception/road_hazards/timing', 1)
        self.lock = open('/tmp/sac_gpu_inference.lock', 'a') if self.p('serialize_gpu') else None
        self.bridge = CvBridge()
        self.info = None
        self.last_run = 0.0
        self.last_depth_receipt = None
        self.pub = self.create_publisher(RoadHazardArray, '/perception/road_hazards', 1)
        self.visualization_pub = (self.create_publisher(
            Image, '/perception/road_hazards/annotated', 1)
            if self.p('publish_visualization') else None)
        if self.p('rgb_only_preview'):
            self.get_logger().warn('RGB preview only: hazard distance and path position are unavailable')
            self.create_subscription(Image, self.p('image_topic'), self.on_rgb,
                                     qos_profile_sensor_data)
        else:
            if self.p('auto_depth_preview'):
                self.get_logger().info('Automatic depth preview: RGB-only until registered depth arrives')
                self.create_subscription(Image, self.p('image_topic'), self.on_rgb,
                                         qos_profile_sensor_data)
                self.create_subscription(Image, self.p('depth_topic'), self.on_depth,
                                         qos_profile_sensor_data)
            self.create_subscription(CameraInfo, self.p('camera_info_topic'),
                                     self.on_info, qos_profile_sensor_data)
            self.rgb = message_filters.Subscriber(self, Image, self.p('image_topic'),
                                                  qos_profile=qos_profile_sensor_data)
            self.depth = message_filters.Subscriber(self, Image, self.p('depth_topic'),
                                                    qos_profile=qos_profile_sensor_data)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [self.rgb, self.depth], queue_size=3, slop=self.p('sync_slop'))
            self.sync.registerCallback(self.on_images)
        self.create_timer(5.0, self.check_input)

    def check_input(self):
        depth_expected = (not self.p('rgb_only_preview') and
                          (not self.p('auto_depth_preview') or self.last_depth_receipt is not None))
        if self.received == 0 or (depth_expected and self.info is None):
            message = ('No ZED RGB frames; check image topic and rosbag /clock' if
                       self.p('rgb_only_preview') or self.p('auto_depth_preview') and
                       self.last_depth_receipt is None else
                       'No synchronized ZED RGB/depth frames; check image, registered depth and CameraInfo topics')
            self.get_logger().warn(message, throttle_duration_sec=5.0)

    def on_info(self, msg):
        self.info = msg

    def on_depth(self, _msg):
        self.last_depth_receipt = time.monotonic()

    def on_rgb(self, rgb):
        if (self.p('rgb_only_preview') or self.last_depth_receipt is None or
                time.monotonic() - self.last_depth_receipt > 1.0):
            self.on_images(rgb, None)

    def on_images(self, rgb, depth):
        self.received += 1
        now = time.monotonic()
        if now - self.last_run < 1.0 / max(.1, self.p('max_fps')):
            return
        self.last_run = now
        age = (self.get_clock().now().nanoseconds -
               rclpy.time.Time.from_msg(rgb.header.stamp).nanoseconds) / 1e9
        if not -self.p('future_stamp_tolerance') <= age <= self.p('max_source_age') or (
                depth is not None and self.info is None):
            return
        try:
            info = self.info
            if depth is not None:
                if (rgb.width, rgb.height) != (depth.width, depth.height) or (
                        rgb.width, rgb.height) != (info.width, info.height):
                    raise ValueError('registered RGB/depth/calibration dimensions must match')
                if not rgb.header.frame_id or rgb.header.frame_id != info.header.frame_id or (
                        depth.header.frame_id != rgb.header.frame_id):
                    raise ValueError('registered optical frames must match')
                if depth.encoding not in ('32FC1', '16UC1'):
                    raise ValueError('depth must be 32FC1 metres or 16UC1 millimetres')
            image = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            depth_array = self.bridge.imgmsg_to_cv2(depth, 'passthrough') if depth is not None else None
            if self.lock:
                fcntl.flock(self.lock, fcntl.LOCK_EX)
            try:
                inference_start = time.perf_counter()
                result = self.model.predict(image, imgsz=self.p('img_size'),
                                            conf=self.p('confidence'), device=self.p('device'),
                                            classes=self.hazard_class_ids,
                                            verbose=False)[0]
            finally:
                if self.lock:
                    fcntl.flock(self.lock, fcntl.LOCK_UN)
            inference_ms = (time.perf_counter()-inference_start)*1000.0
            names = self.names
            output = RoadHazardArray()
            output.header = rgb.header
            for box in result.boxes if result.boxes is not None else []:
                name = names[int(box.cls[0].item())]
                if name not in ('pothole', 'speed_bump'):
                    continue
                hazard = RoadHazard()
                hazard.kind = RoadHazard.POTHOLE if name == 'pothole' else RoadHazard.SPEED_BUMP
                hazard.confidence = float(box.conf[0].item())
                hazard.model_class_id = int(box.cls[0].item())
                hazard.bbox_xyxy = [float(v) for v in box.xyxy[0].tolist()]
                hazard.distance_m = -1.0
                if depth_array is not None:
                    try:
                        xyz, radius = project_box(depth_array, box.xyxy[0].tolist(),
                                                  (info.p[0], info.p[5], info.p[2], info.p[6]),
                                                  .001 if depth.encoding == '16UC1' else 1.0,
                                                  center_fraction=0.6)
                        hazard.position.x, hazard.position.y, hazard.position.z = xyz
                        hazard.radius_m = radius
                        hazard.distance_m = math.sqrt(sum(v*v for v in xyz))
                        hazard.position_valid = True
                    except ValueError:
                        hazard.position_valid = False
                output.hazards.append(hazard)
            self.pub.publish(output)
            if self.visualization_pub is not None:
                annotated = self.bridge.cv2_to_imgmsg(result.plot(), encoding='bgr8')
                annotated.header = rgb.header
                self.visualization_pub.publish(annotated)
            self.sequence += 1
            metric = PipelineTiming()
            metric.stamp = self.get_clock().now().to_msg()
            metric.component = 'road_hazard_inference'
            metric.sequence = self.sequence
            metric.execution_time_ms = inference_ms
            metric.source_age_ms = (self.get_clock().now().nanoseconds -
                rclpy.time.Time.from_msg(rgb.header.stamp).nanoseconds)/1e6
            metric.received_count = self.received
            metric.processed_count = self.sequence
            metric.dropped_count = self.received-self.sequence
            self.timing_pub.publish(metric)
        except Exception as exc:
            self.get_logger().error(f'Hazard frame rejected: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoadHazardNode()
        rclpy.spin(node)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

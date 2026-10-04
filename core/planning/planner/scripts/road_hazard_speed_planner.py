#!/usr/bin/env python3
"""Path-aware speed constraint; independent of lateral control and model IDs."""
import copy
import math
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from sac_interfaces.msg import RoadHazardArray, SpeedConstraint, VehicleSpeed


def project_path(x, y, points):
    best, along, accumulated = float('inf'), 0.0, 0.0
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        ratio = max(0.0, min(1.0, ((x-a[0])*dx+(y-a[1])*dy)/length**2))
        distance = math.hypot(x-a[0]-ratio*dx, y-a[1]-ratio*dy)
        if distance < best:
            best, along = distance, accumulated+ratio*length
        accumulated += length
    return best, along


def approach_speed(distance, crossing_speed, deceleration, buffer_m):
    return math.sqrt(crossing_speed**2 + 2*deceleration*max(0.0, distance-buffer_m))


class HazardSpeedPlanner(Node):
    def __init__(self):
        super().__init__('road_hazard_speed_planner')
        for key, value in {
            'path_topic': '/trajectory_planner/trajectory', 'tracking_frame': 'odom',
            'base_frame': 'base_link', 'source_timeout': .75, 'path_timeout': 1.0,
            'max_speed_mps': 2.0, 'pothole_speed_mps': .3, 'bump_speed_mps': .5,
            'deceleration_mps2': .5, 'buffer_m': 2.0, 'half_width_m': .8,
            'rear_clearance_m': 2.0,
            'future_stamp_tolerance': 0.1,
            'enforce_freshness': True,
        }.items():
            self.declare_parameter(key, value)
        self.p = lambda key: self.get_parameter(key).value
        for key in ('source_timeout', 'path_timeout', 'max_speed_mps',
                    'deceleration_mps2', 'half_width_m', 'rear_clearance_m'):
            if not math.isfinite(self.p(key)) or self.p(key) <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('pothole_speed_mps', 'bump_speed_mps', 'buffer_m'):
            if not math.isfinite(self.p(key)) or self.p(key) < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.path = None
        self.odometry = None
        self.odometry_receipt = None
        self.last_source = None
        self.last_receipt = None
        self.tracks = []
        self.unknown_latched = False
        self.assessment_pub = self.create_publisher(
            RoadHazardArray, '/planning/road_hazard/assessments', 1)
        self.pub = self.create_publisher(SpeedConstraint, '/planning/road_hazard/speed_constraint', 1)
        self.create_subscription(RoadHazardArray, '/perception/road_hazards', self.on_hazards, 1)
        self.create_subscription(VehicleSpeed, '/vehicle/speed', self.on_odometry, 1)
        self.create_subscription(Path, self.p('path_topic'), self.on_path, 1)
        self.create_service(Trigger, '~/reset', self.reset)
        self.create_timer(.05, self.tick)

    def reset(self, request, response):
        # Do not discard the only retained hazard while the vehicle is moving.
        if (self.odometry is None or self.odometry_receipt is None or
                time.monotonic()-self.odometry_receipt > .25 or
                not self.fresh(self.odometry.header.stamp, .25)):
            response.success = False
            response.message = 'Fresh stationary vehicle speed is required'
            return response
        speed = self.odometry.speed_mps
        if not math.isfinite(speed) or speed > .05:
            response.success = False
            response.message = 'Stop vehicle before clearing retained hazards'
            return response
        self.tracks.clear()
        self.unknown_latched = False
        self.last_source = None
        response.success = True
        response.message = 'Reset; waiting for a fresh hazard observation'
        return response

    def on_odometry(self, msg):
        if not self.fresh(msg.header.stamp, .25):
            return
        if self.odometry is not None and Time.from_msg(msg.header.stamp) <= Time.from_msg(
                self.odometry.header.stamp):
            return
        self.odometry = msg
        self.odometry_receipt = time.monotonic()

    def on_path(self, msg):
        self.path = msg

    def fresh(self, stamp, timeout):
        if not self.p('enforce_freshness'):
            return True
        age = (self.get_clock().now()-Time.from_msg(stamp)).nanoseconds / 1e9
        return -self.p('future_stamp_tolerance') <= age <= timeout

    def on_hazards(self, msg):
        if not self.fresh(msg.header.stamp, self.p('source_timeout')):
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_source is not None and stamp <= self.last_source:
            return
        assessment = copy.deepcopy(msg)
        for hazard in assessment.hazards:
            hazard.path_assessed = False
            hazard.intersects_expected_path = False
        try:
            transform = self.tf.lookup_transform(self.p('tracking_frame'), msg.header.frame_id,
                                                 stamp, timeout=Duration(seconds=0.0))
            updates = []
            try:
                path_points = self.path_points()
            except Exception:
                path_points = None
            for hazard in assessment.hazards:
                if not hazard.position_valid:
                    self.unknown_latched = True
                    continue
                values = (hazard.position.x, hazard.position.y, hazard.position.z,
                          hazard.radius_m, hazard.confidence)
                if (not all(math.isfinite(v) for v in values) or hazard.radius_m < 0 or
                        not 0 <= hazard.confidence <= 1 or hazard.kind not in (0, 1)):
                    self.unknown_latched = True
                    continue
                point = PointStamped(header=msg.header, point=hazard.position)
                point = do_transform_point(point, transform).point
                updates.append([point.x, point.y, hazard.radius_m, hazard.kind])
                if path_points is not None:
                    lateral, _ = project_path(point.x, point.y, path_points)
                    hazard.path_assessed = math.isfinite(lateral)
                    hazard.intersects_expected_path = lateral <= self.p('half_width_m')+hazard.radius_m
            for item in updates:
                match = next((t for t in self.tracks if t[3] == item[3] and
                              math.hypot(t[0]-item[0], t[1]-item[1]) < .75), None)
                if match is None:
                    self.tracks.append(item)
                else:
                    match[:] = item
            if len(self.tracks) > 100:
                self.unknown_latched = True
                self.tracks = self.tracks[:100]
            self.last_source = stamp
            self.last_receipt = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f'Hazard transform unavailable: {exc}', throttle_duration_sec=5.0)
        finally:
            self.assessment_pub.publish(assessment)

    def path_points(self):
        if self.path is None or len(self.path.poses) < 2 or not self.fresh(
                self.path.header.stamp, self.p('path_timeout')):
            raise ValueError('path unavailable or stale')
        fixed = self.p('tracking_frame')
        path_tf = self.tf.lookup_transform(fixed, self.path.header.frame_id,
                                          Time.from_msg(self.path.header.stamp))
        points = []
        for pose in self.path.poses[:2000]:
            p = do_transform_point(PointStamped(header=self.path.header,
                                                 point=pose.pose.position), path_tf).point
            if not math.isfinite(p.x) or not math.isfinite(p.y):
                raise ValueError('invalid path geometry')
            points.append((p.x, p.y))
        if not any(math.hypot(b[0]-a[0], b[1]-a[1]) > 1e-6
                   for a, b in zip(points, points[1:])):
            raise ValueError('degenerate path')
        return points

    def compute(self):
        if self.unknown_latched:
            return 0.0, 'unresolved hazard depth; operator reset required'
        if (self.last_source is None or self.last_receipt is None or
                (self.p('enforce_freshness') and
                 time.monotonic()-self.last_receipt > self.p('source_timeout')) or
                not self.fresh(self.last_source.to_msg(), self.p('source_timeout'))):
            return 0.0, 'hazard observations unavailable or stale'
        if self.path is None or len(self.path.poses) < 2 or not self.fresh(
                self.path.header.stamp, self.p('path_timeout')):
            return 0.0, 'path unavailable or stale'
        fixed = self.p('tracking_frame')
        base_tf = self.tf.lookup_transform(fixed, self.p('base_frame'), Time())
        # Reject frozen TF, except a static transform (stamp zero).
        if (base_tf.header.stamp.sec or base_tf.header.stamp.nanosec) and not self.fresh(
                base_tf.header.stamp, self.p('source_timeout')):
            return 0.0, 'vehicle transform stale'
        points = self.path_points()
        origin = base_tf.transform.translation
        q = base_tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        _, ego_s = project_path(origin.x, origin.y, points)
        cap = self.p('max_speed_mps')
        kept = []
        for x, y, radius, kind in self.tracks:
            forward = math.cos(yaw)*(x-origin.x)+math.sin(yaw)*(y-origin.y)
            # Retain missed detections until the complete vehicle has passed them.
            if forward < -self.p('rear_clearance_m')-radius:
                continue
            kept.append([x, y, radius, kind])
            lateral, hazard_s = project_path(x, y, points)
            beside = abs(-math.sin(yaw)*(x-origin.x)+math.cos(yaw)*(y-origin.y))
            crossing_vehicle = (forward <= radius and
                                beside <= self.p('half_width_m')+radius)
            if lateral > self.p('half_width_m')+radius and not crossing_vehicle:
                continue
            crossing = self.p('pothole_speed_mps' if kind == 0 else 'bump_speed_mps')
            distance = 0.0 if crossing_vehicle else hazard_s-ego_s-radius
            cap = min(cap, approach_speed(distance, crossing,
                                         self.p('deceleration_mps2'), self.p('buffer_m')))
        self.tracks = kept
        return cap, 'road hazard approach/crossing' if cap < self.p('max_speed_mps') else 'clear'

    def tick(self):
        try:
            cap, reason = self.compute()
        except Exception as exc:
            cap, reason = 0.0, f'geometry unavailable: {exc}'
        msg = SpeedConstraint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.max_speed_mps, msg.reason = float(cap), reason
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = HazardSpeedPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Stamped STOP/red lifecycle; legacy /stop remains a separate manual latch."""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sac_interfaces.msg import CameraDetection, CameraDetectionArray, SpeedConstraint, VehicleSpeed
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class TrafficSpeedPlanner(Node):
    def __init__(self):
        super().__init__('traffic_speed_planner')
        for key, value in {'max_speed_mps': 2.0, 'camera_timeout': .75,
                           'enforce_freshness': True,
                           'odometry_timeout': .25, 'confidence': .5,
                           'future_stamp_tolerance': .1,
                           'stop_distance_m': 10.0, 'reset_clear_seconds': 1.0,
                           'stop_hold_seconds': 3.0, 'stop_rearm_clear_seconds': 2.0,
                           'green_confirm_frames': 3,
                           'enable_stop_sign': True,
                           'speed_topic': '/vehicle/speed'}.items():
            self.declare_parameter(key, value)
        self.p = lambda k: self.get_parameter(k).value
        for key in ('max_speed_mps', 'camera_timeout', 'odometry_timeout',
                    'stop_distance_m', 'reset_clear_seconds',
                    'stop_hold_seconds', 'stop_rearm_clear_seconds'):
            if not math.isfinite(self.p(key)) or self.p(key) <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not 0 <= self.p('confidence') <= 1:
            raise ValueError('confidence must be in [0,1]')
        if not 0 <= self.p('future_stamp_tolerance') <= .5:
            raise ValueError('future_stamp_tolerance must be within [0, 0.5] seconds')
        if self.p('green_confirm_frames') < 2:
            raise ValueError('green_confirm_frames must be at least 2')
        self.camera_stamp = None
        self.camera_receipt = None
        self.odom_stamp = None
        self.odom_receipt = None
        self.vehicle_speed_receipt = None
        self.stationary_since = None
        self.clear_since = None
        self.last_legacy_stop = False
        self.latched = False
        self.reason = ''
        self.stop_state = 'CRUISE'
        self.red_latched = False
        self.green_frames = 0
        self.stop_suppressed = False
        self.stop_absent_since = None
        self.create_subscription(CameraDetectionArray, '/yolo_detections', self.on_camera, 1)
        self.create_subscription(Bool, '/stop', self.on_stop, 10)
        if self.p('speed_topic'):
            self.create_subscription(VehicleSpeed, self.p('speed_topic'), self.on_odometry, 1)
        else:
            self.get_logger().warn('Using EKF odometry as speed feedback')
        self.create_subscription(Odometry, '/localization/ekf_localizer/odom',
                                 self.on_ekf_odometry, 1)
        self.pub = self.create_publisher(SpeedConstraint, '/planning/traffic/speed_constraint', 1)
        self.create_service(Trigger, '~/reset', self.reset)
        self.create_timer(.05, self.tick)

    def fresh(self, stamp, receipt, timeout):
        if stamp is None or receipt is None:
            return False
        if not self.p('enforce_freshness'):
            return True
        age = (self.get_clock().now()-stamp).nanoseconds/1e9
        return -self.p('future_stamp_tolerance') <= age <= timeout and time.monotonic()-receipt <= timeout

    def on_stop(self, msg):
        self.last_legacy_stop = msg.data
        if msg.data:
            self.latched = True
            self.reason = 'legacy /stop request; operator clearance required'
            self.clear_since = None

    def on_camera(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds/1e9
        if (self.p('enforce_freshness') and
                not -self.p('future_stamp_tolerance') <= age <= self.p('camera_timeout')) or (
                self.camera_stamp is not None and stamp <= self.camera_stamp):
            return
        if not self.fresh(self.camera_stamp, self.camera_receipt, self.p('camera_timeout')):
            self.clear_since = None
        self.camera_stamp, self.camera_receipt = stamp, time.monotonic()
        stop = False
        red = False
        green = False
        for detection in msg.detections:
            if not math.isfinite(detection.confidence) or detection.confidence < self.p('confidence'):
                continue
            if detection.label not in (CameraDetection.RED_LIGHT, CameraDetection.STOP,
                                       CameraDetection.GREEN_LIGHT):
                continue
            # Missing depth never authorizes continuing past a visible stop indication.
            if (not math.isfinite(detection.distance) or detection.distance <= 0 or
                    detection.distance <= self.p('stop_distance_m')):
                if detection.label == CameraDetection.STOP and self.p('enable_stop_sign'):
                    stop = True
                elif detection.label == CameraDetection.RED_LIGHT:
                    red = True
                else:
                    green = True
        if red:
            self.red_latched = True
            self.green_frames = 0
        elif self.red_latched:
            self.green_frames = self.green_frames + 1 if green else 0
            if (self.green_frames >= self.p('green_confirm_frames') and
                    self.fresh(self.odom_stamp, self.odom_receipt,
                               self.p('odometry_timeout'))):
                self.red_latched = False
                self.green_frames = 0
        if stop and not self.stop_suppressed and self.stop_state == 'CRUISE':
            self.stop_state = 'APPROACH'
            self.stationary_since = None
        if not stop:
            if self.stop_absent_since is None:
                self.stop_absent_since = time.monotonic()
            elif (self.stop_suppressed and
                  time.monotonic()-self.stop_absent_since >= self.p('stop_rearm_clear_seconds')):
                self.stop_suppressed = False
        else:
            self.stop_absent_since = None
        if not stop and not red:
            if not self.last_legacy_stop and self.clear_since is None:
                self.clear_since = time.monotonic()

    def on_odometry(self, msg):
        if self.accept_speed(msg.header.stamp, msg.speed_mps):
            self.vehicle_speed_receipt = time.monotonic()

    def on_ekf_odometry(self, msg):
        if (self.vehicle_speed_receipt is not None and
                time.monotonic() - self.vehicle_speed_receipt <= self.p('odometry_timeout')):
            return
        self.accept_speed(msg.header.stamp, msg.twist.twist.linear.x)

    def accept_speed(self, stamp_msg, speed):
        stamp = Time.from_msg(stamp_msg)
        age = (self.get_clock().now()-stamp).nanoseconds/1e9
        if (self.p('enforce_freshness') and
                not -self.p('future_stamp_tolerance') <= age <= self.p('odometry_timeout')) or (
                self.odom_stamp is not None and stamp <= self.odom_stamp):
            return False
        if not math.isfinite(speed):
            self.stationary_since = None
            self.odom_receipt = None
            return False
        # A gap in feedback cannot count toward a continuously stationary interval.
        if not self.fresh(self.odom_stamp, self.odom_receipt, self.p('odometry_timeout')):
            self.stationary_since = None
        self.odom_stamp, self.odom_receipt = stamp, time.monotonic()
        if speed > .05:
            self.stationary_since = None
        elif self.stationary_since is None:
            self.stationary_since = time.monotonic()
        if self.stop_state == 'APPROACH' and speed <= .05:
            self.stop_state = 'HOLD'
        return True

    def reset(self, request, response):
        now = time.monotonic()
        ready = (self.fresh(self.camera_stamp, self.camera_receipt, self.p('camera_timeout')) and
                 self.fresh(self.odom_stamp, self.odom_receipt, self.p('odometry_timeout')) and
                 not self.last_legacy_stop and self.clear_since is not None and
                 self.stationary_since is not None and
                 now-self.clear_since >= self.p('reset_clear_seconds') and
                 now-self.stationary_since >= self.p('reset_clear_seconds'))
        response.success = bool(ready)
        response.message = ('Stop cleared by operator' if ready else
                            'Requires fresh clear camera, cleared /stop and stationary feedback')
        if ready:
            self.latched, self.reason = False, ''
            self.red_latched = False
            self.stop_state = 'CRUISE'
            self.stop_suppressed = True
        return response

    def constraint(self):
        if not self.fresh(self.camera_stamp, self.camera_receipt, self.p('camera_timeout')):
            self.clear_since = None
            return 0.0, 'camera unavailable or stale'
        if self.latched:
            return 0.0, self.reason
        if self.red_latched:
            return 0.0, 'red light until stable green'
        if self.stop_state != 'CRUISE':
            if (self.stop_state == 'HOLD' and
                    self.fresh(self.odom_stamp, self.odom_receipt, self.p('odometry_timeout')) and
                    self.stationary_since is not None and
                    time.monotonic()-self.stationary_since >= self.p('stop_hold_seconds')):
                self.stop_state = 'CRUISE'
                self.stop_suppressed = True
            else:
                return 0.0, f'STOP {self.stop_state.lower()}'
        return self.p('max_speed_mps'), 'clear'

    def tick(self):
        msg = SpeedConstraint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.max_speed_mps, msg.reason = self.constraint()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TrafficSpeedPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

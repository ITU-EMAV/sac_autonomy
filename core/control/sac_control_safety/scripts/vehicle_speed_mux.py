#!/usr/bin/env python3
"""Expose real wheel or EKF speed without inventing vehicle odometry."""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sac_interfaces.msg import VehicleSpeed
from std_msgs.msg import Float32


class VehicleSpeedMux(Node):
    def __init__(self):
        super().__init__('vehicle_speed_mux')
        self.declare_parameter('encoder_topic', '/encoder_speed')
        self.declare_parameter('ekf_odom_topic', '/localization/ekf_localizer/odom')
        self.declare_parameter('output_topic', '/vehicle/speed')
        self.declare_parameter('encoder_timeout', .15)
        self.declare_parameter('ekf_timeout', .15)
        self.declare_parameter('allow_ekf_fallback', False)
        self.encoder = None
        self.encoder_receipt = None
        self.ekf = None
        self.ekf_receipt = None
        self.pub = self.create_publisher(VehicleSpeed,
            self.get_parameter('output_topic').value, 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Float32, self.get_parameter('encoder_topic').value,
                                 self.on_encoder, qos)
        self.create_subscription(Odometry, self.get_parameter('ekf_odom_topic').value,
                                 self.on_ekf, qos)
        self.create_timer(.02, self.tick)

    def on_encoder(self, msg):
        if math.isfinite(msg.data) and msg.data >= 0:
            self.encoder = float(msg.data)
            self.encoder_receipt = time.monotonic()

    def on_ekf(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        speed = msg.twist.twist.linear.x
        if (math.isfinite(speed) and speed >= 0 and
                0 <= age <= self.get_parameter('ekf_timeout').value):
            self.ekf = msg
            self.ekf_receipt = time.monotonic()

    def tick(self):
        now = time.monotonic()
        msg = VehicleSpeed()
        if (self.encoder_receipt is not None and
                now-self.encoder_receipt <= self.get_parameter('encoder_timeout').value):
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.speed_mps = self.encoder
            msg.source_stamp_valid = False  # Float32 has no acquisition timestamp.
            msg.source_topic = self.get_parameter('encoder_topic').value
        elif (self.get_parameter('allow_ekf_fallback').value and
              self.ekf_receipt is not None and
              now-self.ekf_receipt <= self.get_parameter('ekf_timeout').value and
              0 <= (self.get_clock().now()-Time.from_msg(self.ekf.header.stamp)).nanoseconds*1e-9 <=
              self.get_parameter('ekf_timeout').value):
            msg.header = self.ekf.header
            msg.speed_mps = self.ekf.twist.twist.linear.x
            msg.source_stamp_valid = True
            msg.source_topic = self.get_parameter('ekf_odom_topic').value
        else:
            return  # Consumers fail closed on silence.
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = VehicleSpeedMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

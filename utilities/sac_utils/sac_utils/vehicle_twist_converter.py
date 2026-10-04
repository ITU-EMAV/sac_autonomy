#!/usr/bin/env python3
"""Arac hizini `geometry_msgs/TwistWithCovarianceStamped`'e cevirir.

gyro_odometer (ve dolayisiyla ekf_localizer) arac boylamsal hizini
TwistWithCovarianceStamped olarak bekler. Bu dugum iki girdi tipini destekler:

  input_type: "float32"  -> std_msgs/Float32   (or. /encoder_speed, m/s)
  input_type: "odometry" -> nav_msgs/Odometry  (twist alani kullanilir)

Cikis sadece boylamsal hizi (vx) tasir; aci hizi gyro_odometer tarafindan
IMU'dan eklenir. Bu yuzden aci hizi varyansi buyuk verilir.
"""

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

LARGE_VARIANCE = 1000.0


class VehicleTwistConverter(Node):

    def __init__(self):
        super().__init__('vehicle_twist_converter')

        self.declare_parameter('input_type', 'float32')
        self.declare_parameter('input_topic', '/encoder_speed')
        self.declare_parameter('output_topic', '/vehicle/twist_with_covariance')
        self.declare_parameter('output_frame_id', 'base_link')
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('linear_variance', 0.04)
        self.declare_parameter('angular_variance', LARGE_VARIANCE)

        gp = self.get_parameter
        self.input_type = gp('input_type').value.lower()
        input_topic = gp('input_topic').value
        output_topic = gp('output_topic').value
        self.frame_id = gp('output_frame_id').value
        self.scale = gp('speed_scale').value
        self.lin_var = gp('linear_variance').value
        self.ang_var = gp('angular_variance').value

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(TwistWithCovarianceStamped, output_topic, 10)

        if self.input_type == 'float32':
            self.sub = self.create_subscription(
                Float32, input_topic, self.float32_callback, qos)
        elif self.input_type == 'odometry':
            self.sub = self.create_subscription(
                Odometry, input_topic, self.odom_callback, qos)
        else:
            raise ValueError(
                f"invalid input_type '{self.input_type}'; expected 'float32' or 'odometry'")

        self.get_logger().info(
            f'vehicle_twist_converter [{self.input_type}]: {input_topic} -> {output_topic}')

    def _base_msg(self, stamp):
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        cov = [0.0] * 36
        cov[0] = self.lin_var            # vx
        cov[7] = LARGE_VARIANCE          # vy  (olculmuyor)
        cov[14] = LARGE_VARIANCE         # vz  (olculmuyor)
        cov[21] = LARGE_VARIANCE         # wx
        cov[28] = LARGE_VARIANCE         # wy
        cov[35] = self.ang_var           # wz (gyro_odometer dolduracak)
        msg.twist.covariance = cov
        return msg

    def float32_callback(self, msg: Float32):
        # Float32'de zaman damgasi yok; mevcut saat kullanilir.
        # use_sim_time true iken bu bag saatidir.
        out = self._base_msg(self.get_clock().now().to_msg())
        out.twist.twist.linear.x = float(msg.data) * self.scale
        self.pub.publish(out)

    def odom_callback(self, msg: Odometry):
        out = self._base_msg(msg.header.stamp)
        out.twist.twist.linear.x = msg.twist.twist.linear.x * self.scale
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleTwistConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

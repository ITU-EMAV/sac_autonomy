#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3

class MagVisualizer(Node):
    def __init__(self):
        super().__init__('magnetometer_visualizer')
        self.sub = self.create_subscription(
            MagneticField, '/zed/zed_node/imu/mag', self.cb_mag, 10)
        self.pub = self.create_publisher(Marker, '/mag_field_marker', 10)

    def cb_mag(self, msg: MagneticField):
        # create an arrow Marker
        m = Marker()
        m.header = msg.header
        m.ns = 'mag_field'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        # arrow from origin to (x,y,z)
        start = Point(x=0.0, y=0.0, z=0.0)
        end   = Point(x=msg.magnetic_field.x*1e5,
                      y=msg.magnetic_field.y*1e5,
                      z=msg.magnetic_field.z*1e5)
        m.points = [start, end]
        # thickness of arrow: (shaft_diameter, head_diameter, head_length)
        m.scale = Vector3(x=0.001, y=0.001, z=0.01)
        # color: RGBA (green)
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        # lifetime: how long it stays (0 = forever until replaced)
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        self.pub.publish(m)

if __name__ == '__main__':
    rclpy.init()
    node = MagVisualizer()
    print('node created')
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class LocalPathPublisher(Node):
    def __init__(self):
        super().__init__('local_path_publisher')
        self.publisher_ = self.create_publisher(Path, 'planner/local_plan', 10)
        self.timer = self.create_timer(1.0, self.publish_path)  # Publish at 1 Hz

    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"  # Change to the correct frame

        # Define a simple path (example: semicircle)
        num_points = 20
        angles = np.linspace(0, np.pi, num_points)
        path_x = np.cos(angles)
        path_y = np.sin(angles)

        for x, y in zip(path_x, path_y):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0  # Assuming a 2D path
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info("Published local path")

def main(args=None):
    rclpy.init(args=args)
    node = LocalPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

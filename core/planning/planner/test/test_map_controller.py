"""Controller minimum selection with actual ROS SpeedConstraint inputs."""
import os
import subprocess
import time
import unittest

import rclpy
from rclpy.duration import Duration
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sac_interfaces.msg import ActuatorCommand, SpeedConstraint


class MapControllerTest(unittest.TestCase):
    def test_map_traffic_hazard_minimum(self):
        rclpy.init()
        node = rclpy.create_node('map_controller_test')
        received = []
        node.create_subscription(ActuatorCommand, '/test/map_command', received.append, 10)
        path_pub = node.create_publisher(Path, '/test/map_path', 1)
        pubs = {kind: node.create_publisher(SpeedConstraint,
            f'/planning/{kind}/speed_constraint', 1)
            for kind in ('map', 'traffic', 'road_hazard')}
        binary = os.path.join(get_package_prefix('planner'), 'lib', 'planner', 'controller_node')
        process = subprocess.Popen([binary, '--ros-args',
            '-p', 'local_path_mode:=true', '-p', 'global_frame:=base_link',
            '-p', 'trajectory_topic:=/test/map_path',
            '-p', 'command_topic:=/test/map_command',
            '-p', 'require_map_speed_constraint:=true',
            '-p', 'require_traffic_speed_constraint:=true',
            '-p', 'require_hazard_speed_constraint:=true',
            '-p', 'linear_velocity:=2.0', '-p', 'min_linear_velocity:=1.0'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        def drive(duration, map_cap, traffic_cap=2.0, hazard_cap=2.0,
                  stamp_ahead=0.0):
            received.clear()
            until = time.monotonic()+duration
            while time.monotonic() < until:
                stamp = (node.get_clock().now() + Duration(seconds=stamp_ahead)).to_msg()
                path = Path()
                path.header.frame_id = 'base_link'
                path.header.stamp = stamp
                for x in range(20):
                    point = PoseStamped()
                    point.pose.position.x = float(x)
                    point.pose.orientation.w = 1.0
                    path.poses.append(point)
                path_pub.publish(path)
                for kind, speed in (('map', map_cap), ('traffic', traffic_cap),
                                    ('road_hazard', hazard_cap)):
                    if speed is None:
                        continue
                    cap = SpeedConstraint()
                    cap.header.stamp = stamp
                    cap.max_speed_mps = speed
                    pubs[kind].publish(cap)
                rclpy.spin_once(node, timeout_sec=.025)
            self.assertIsNone(process.poll())
            self.assertTrue(received)
            return received[-1].target_speed_mps

        try:
            self.assertAlmostEqual(drive(1.4, .3), .3, places=2)
            self.assertAlmostEqual(drive(.5, .3, hazard_cap=.2), .2, places=2)
            self.assertGreater(drive(.5, .3, stamp_ahead=.025), 0.0)
            self.assertEqual(drive(.35, .3, traffic_cap=0.), 0.)
            self.assertEqual(drive(.35, None), 0.)
            self.assertEqual(drive(.35, float('nan')), 0.)
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()

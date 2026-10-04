import math
import socket
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sac_interfaces.msg import ActuatorCommand, CommandGuardianStatus, VehicleSpeed
from std_srvs.srv import SetBool


def generate_test_description():
    guardian = launch_ros.actions.Node(
        package="sac_control_safety", executable="command_guardian",
        name="command_guardian_test", parameters=[{
            "allow_vehicle_output": True,
            "require_collision_perception": False,
            "require_camera_behavior": False,
            "vehicle_feedback_max_age_ms": 50.0,
            "localization_max_age_ms": 100.0,
            "trajectory_max_age_ms": 250.0,
            "command_max_age_ms": 100.0,
            "output_period_ms": 10,
        }])
    transport = launch_ros.actions.Node(
        package="sac_network", executable="udp_sender",
        name="udp_transport_test", parameters=[{
            "transport_enabled": False,
            "gateway_supports_safety_contract": False,
            "gateway_address": "127.0.0.1",
            "gateway_port": 14950,
        }])
    return launch.LaunchDescription([
        guardian, transport, launch_testing.actions.ReadyToTest()
    ]), {"guardian": guardian}


class GuardianSilTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("guardian_sil_test")
        qos = 1
        cls.command_pub = cls.node.create_publisher(
            ActuatorCommand, "/control/controller_command", qos)
        cls.pose_pub = cls.node.create_publisher(
            PoseWithCovarianceStamped, "/localization/online/pose", qos)
        cls.feedback_pub = cls.node.create_publisher(
            VehicleSpeed, "/vehicle/speed", qos)
        cls.path_pub = cls.node.create_publisher(Path, "/planning/trajectory", qos)
        cls.outputs = []
        cls.status = []
        cls.node.create_subscription(
            ActuatorCommand, "/vehicle/actuator_command", cls.outputs.append, qos)
        cls.node.create_subscription(
            CommandGuardianStatus, "/safety/command_guardian/status", cls.status.append, qos)
        cls.arm = cls.node.create_client(SetBool, "/safety/command_guardian/arm")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def arm_guardian(self, value):
        self.assertTrue(self.arm.wait_for_service(timeout_sec=3.0))
        future = self.arm.call_async(SetBool.Request(data=value))
        end = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(future.done())

    def publish_inputs(self):
        stamp = self.node.get_clock().now().to_msg()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = stamp
        feedback = VehicleSpeed()
        feedback.header.stamp = stamp
        path = Path()
        path.header.stamp = stamp
        path.poses.append(__import__("geometry_msgs.msg", fromlist=["PoseStamped"]).PoseStamped())
        self.pose_pub.publish(pose)
        self.feedback_pub.publish(feedback)
        self.path_pub.publish(path)

    def command(self, sequence, speed=0.0):
        msg = ActuatorCommand()
        msg.source_stamp = self.node.get_clock().now().to_msg()
        msg.publication_stamp = msg.source_stamp
        msg.source_steady_time_ns = time.monotonic_ns()
        msg.sequence = sequence
        msg.validity_duration.nanosec = 100_000_000
        msg.mode = ActuatorCommand.MODE_AUTONOMOUS
        msg.armed = True
        msg.enable = True
        msg.target_speed_mps = speed
        msg.validity_flags = (ActuatorCommand.VALID_STEERING |
                              ActuatorCommand.VALID_THROTTLE |
                              ActuatorCommand.VALID_BRAKE |
                              ActuatorCommand.VALID_TARGET_SPEED |
                              ActuatorCommand.INTEGRITY_OK)
        msg.source_id = "planner_controller"
        return msg

    def test_guardian_fault_matrix(self):
        udp_probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_probe.bind(("127.0.0.1", 14950))
        udp_probe.settimeout(0.05)
        self.spin_for(0.2)
        self.assertTrue(self.outputs)
        self.assertFalse(self.outputs[-1].enable)
        self.assertEqual(self.outputs[-1].target_speed_mps, 0.0)

        self.arm_guardian(True)
        self.publish_inputs()
        self.command_pub.publish(self.command(1))
        self.spin_for(0.04)
        self.assertEqual(self.status[-1].state, CommandGuardianStatus.STATE_ACTIVE)

        # Controller crash/expiration: output must become a newly generated stop.
        self.spin_for(0.15)
        self.assertFalse(self.outputs[-1].enable)
        self.assertEqual(self.outputs[-1].brake_normalized, 1.0)
        self.assertEqual(self.status[-1].state, CommandGuardianStatus.STATE_STOPPING)

        # Reset, then prove duplicate/out-of-order sequence latches FAULT.
        self.arm_guardian(False)
        self.arm_guardian(True)
        self.publish_inputs()
        self.command_pub.publish(self.command(2))
        self.spin_for(0.03)
        self.command_pub.publish(self.command(2))
        self.spin_for(0.03)
        self.assertEqual(self.status[-1].state, CommandGuardianStatus.STATE_FAULT)
        self.assertFalse(self.outputs[-1].enable)

        # Reset, then prove non-finite command rejection.
        self.arm_guardian(False)
        self.arm_guardian(True)
        self.publish_inputs()
        invalid = self.command(3)
        invalid.steering_angle_rad = math.nan
        self.command_pub.publish(invalid)
        self.spin_for(0.03)
        self.assertEqual(self.status[-1].state, CommandGuardianStatus.STATE_FAULT)
        self.assertIn("NaN", self.status[-1].fault_reason)

        # Delayed source timestamp is rejected even if just received by DDS.
        self.arm_guardian(False)
        self.arm_guardian(True)
        self.publish_inputs()
        delayed = self.command(4)
        delayed.source_stamp.sec -= 1
        self.command_pub.publish(delayed)
        self.spin_for(0.03)
        self.assertEqual(self.status[-1].state, CommandGuardianStatus.STATE_FAULT)
        self.assertIn("stale ROS", self.status[-1].fault_reason)

        # A second publisher violates the single-authority invariant.
        self.arm_guardian(False)
        self.arm_guardian(True)
        duplicate_pub = self.node.create_publisher(
            ActuatorCommand, "/control/controller_command", 1)
        self.spin_for(0.1)
        self.command_pub.publish(self.command(5))
        self.spin_for(0.03)
        self.assertEqual(self.status[-1].state, CommandGuardianStatus.STATE_FAULT)
        self.assertIn("exactly one publisher", self.status[-1].fault_reason)
        self.node.destroy_publisher(duplicate_pub)

        # Even guardian output cannot reach UDP with Phase-1 transport defaults.
        with self.assertRaises(socket.timeout):
            udp_probe.recvfrom(4096)
        udp_probe.close()

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, TransformStamped
from tracker_classes import StanleyController, PurePursuitController, VehicleState

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker_node")

        # -----------------------------
        #  ROS 2 Components
        # -----------------------------
        self.callback_group = ReentrantCallbackGroup()

        # Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_publisher = self.create_publisher(Path, "/plan_debug", 10)

        # Subscriptions
        self.create_subscription(Odometry, "/odom", self.pose_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Path, "/trajectory_planner/trajectory", self.path_callback, 10, callback_group=self.callback_group)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/pcl_pose",
            self.loc_cb,
            10,
            callback_group=self.callback_group
        )

        # -----------------------------
        # Internal State Variables
        # -----------------------------
        self.path = None
        self.current_pose = None

        self.get_logger().info("Tracker Node Initialized")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def pose_callback(self, msg: Odometry):
        """ Updates current vehicle pose. """
        # self.current_pose = msg.pose.pose

        # Extract linear velocity components
        vx = msg.twist.twist.linear.x  # east speed
        vy = msg.twist.twist.linear.y  # north speed
        self.current_speed = np.sqrt(vx**2 + vy**2)

    def path_callback(self, msg: Path):
        """ Receives planned path in odom frame """
        self.path = msg

    def loc_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

if __name__ == "__main__":

    # -----------------------------
    #  Parameters
    # -----------------------------
    look_aheah_gain = 0.1  # Look-ahead gain
    look_ahead_distance = 2.0  # [m] Look-ahead distance
    dt = 0.1  # [s] Time tick
    wheelbase = 1.8  # [m] Wheelbase of vehicle
    use_stanley = False  # Set to True to use Stanley controller instead of Pure Pursuit
    controller_frequency = 10
    
    # -----------------------------
    #  Initialize ROS 2
    # -----------------------------
    rclpy.init()
    node = TrackerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # -----------------------------
    #  Controller Initialization
    # -----------------------------
    if use_stanley:
        controller = StanleyController(wheel_base=wheelbase)
    else:
        controller = PurePursuitController(wheel_base=wheelbase, look_ahead_gain=look_aheah_gain, look_ahead_distance=look_ahead_distance)
    
    # Generate example path
    path_x = np.arange(0, 100, 0.5)
    path_y = np.array([np.sin(ix / 5.0) * ix / 2.0 for ix in path_x])

    # path_x = 10*np.cos(np.arange(0, 50, 0.5)/5)
    # path_y = 10*np.sin(np.arange(0, 50, 0.5)/5)
    

    target_speed = 3.0  # [m/s]
    max_simulation_time = 100.0  # [s]



    def track():
        """ Compute steering angle and send control commands. """
        if node.path is None or node.current_pose is None:
            node.cmd_vel_publisher.publish(Twist())  # Stop the vehicle
            return
        
        # Extract vehicle position & heading
        p = node.current_pose.position
        o = node.current_pose.orientation
        yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        print("yaw:", np.degrees(yaw))
        linear_velocity = 1.0 # aads



        vehicle_state = VehicleState(x=p.x, y=p.y, yaw=yaw, v=linear_velocity, wheel_base=wheelbase)

        # create the path 
        path_x = np.array(
            [pose.pose.position.x for pose in node.path.poses]
        )
        path_y = np.array(
            [pose.pose.position.y for pose in node.path.poses]
        )

        # Compute Steering Angle
        steering_angle = controller.compute_steering_angle(vehicle_state, path_x, path_y)
        steering_angle = np.clip(steering_angle, -0.35, 0.35)  # Limit to ±20 degrees

        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = steering_angle
        node.cmd_vel_publisher.publish(twist)

        # node.get_logger().info(f" Steering Angle: {steering_angle:.3f}, Time: {time.time() - t1:.4f} sec")


    # -----------------------------
    #  Timer for Control Loop
    # -----------------------------
    node.create_timer(1 / controller_frequency, track)

    # -----------------------------
    #  Start Execution
    # -----------------------------
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
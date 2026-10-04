import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Point,
    PointStamped,
    Quaternion,
    Twist,
    Pose,
    PoseArray,
)

from visualization_msgs.msg import Marker
from tf_transformations import (
    quaternion_from_euler,
    quaternion_multiply,
    euler_from_quaternion,
)
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sac_utils.rotation_utils import transform_path

rclpy.init()

# parameters
linear_velocity = 1.0
wheelbase = 1.8
controller_frequency = 10
global_frame = "odom"


node = Node(node_name="tracker_node")
executor = MultiThreadedExecutor()
executor.add_node(node)
callback_group = ReentrantCallbackGroup()

tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

path: Path = None
current_pose: Pose = None
grid: OccupancyGrid = None

cmd_vel_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
marker_publisher = node.create_publisher(Marker, "/plan_marker", 10)

path_publisher = node.create_publisher(Path, "/plan_debug", 10)


def track():

    # if path is None or current_pose is None or grid is None:
    if path is None or current_pose is None:
        # stop the car
        cmd_vel_publisher.publish(Twist())
        return
    
    t1 = time.time()

    # get the position and orientation of the center of the front wheels
    p = current_pose.position
    o = current_pose.orientation
    yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
    f_x, f_y = move_point_along_line([p.x, p.y], yaw, 0.9)
    curr = np.array([f_x, f_y])

    # create the path np array
    path_arr = np.array(
        [[pose.pose.position.x, pose.pose.position.y] for pose in path.poses]
    )

    # check if the path is empty
    if len(path_arr) == 0:
        node.get_logger().info("Path is empty")
        return

    # get the closest point on the path
    # print()
    # print(curr)
    # print()
    # print(path_arr)
    nearest_id = np.argmin(np.sum((curr - path_arr) ** 2, axis=1))

    if nearest_id == len(path_arr) - 1:
        # destination reached, stop the robot
        node.get_logger().info("Destination reached! Stopping the robot.")
        cmd_vel_publisher.publish(Twist())
        return

    lookahead_index = nearest_id + 5
    if lookahead_index >= len(path_arr):
        lookahead_index = len(path_arr) - 1

    lookahead_point = path_arr[lookahead_index]

    # Calculate angle to lookahead waypoint
    alpha = np.arctan2(lookahead_point[1] - curr[1], lookahead_point[0] - curr[0]) - yaw
    lookahead_distance = np.hypot(
        lookahead_point[1] - curr[1], lookahead_point[0] - curr[0]
    )
    steering_angle = np.arctan2(2 * wheelbase * np.sin(alpha), lookahead_distance)
    
    # saturate the steering angle between += 0.35 rad (20deg)
    steering_angle = np.clip(steering_angle,-0.35, 0.35)

    # angular_velocity = steering_angle_to_angular_velocity(steering_angle)
    angular_velocity = steering_angle
    print(
        f"alpha: {alpha} | lookahead_distance: {lookahead_distance} | steering_angle: {steering_angle} | angular_velocity: {angular_velocity}"
    )

    # publish the message
    twist = Twist()
    vel_x = (
        linear_velocity if linear_velocity > lookahead_distance else lookahead_distance
    )
    twist.linear.x = 0.0 # float(vel_x) # DEBUG ONLY
    twist.angular.z = angular_velocity
    cmd_vel_publisher.publish(twist)
    node.get_logger().info(f"Time taken: {time.time() - t1}")



def steering_angle_to_angular_velocity(steering_angle):
    angular_velocity = np.tan(steering_angle) * linear_velocity / wheelbase
    return float(angular_velocity)


def pose_callback(msg: Odometry):
    global current_pose
    current_pose = msg.pose.pose


def move_point_along_line(point, angle, distance):
    new_x = point[0] + distance * np.cos(angle)
    new_y = point[1] + distance * np.sin(angle)
    return new_x, new_y


def path_callback(msg):
    if msg.header.frame_id != global_frame:
        node.get_logger().info(f"The goal arrived from the frame {msg.header.frame_id}")
        try:
            t = tf_buffer.lookup_transform(
                global_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            msg = transform_path(msg, t)
            msg.header.frame_id = global_frame
            node.get_logger().info(f"Transformation found, transforming the goal...")

        except TransformException as ex:
            node.get_logger().info(
                f"Could not transform {msg.header.frame_id} to base_link: {ex}"
            )
            return
    path_publisher.publish(msg)
    global path
    path = msg


def occupancy_grid_callback(msg):
    global grid
    grid = msg


# pose sub
node.create_subscription(
    Odometry, "odometry/filtered", pose_callback, 10, callback_group=callback_group
)

# path sub
node.create_subscription(Path, "planner/local_plan", path_callback, 10, callback_group=callback_group)

# occupancy grid sub
node.create_subscription(
    OccupancyGrid,
    "occupancy_grid",
    occupancy_grid_callback,
    10,
    callback_group=callback_group,
)
controller_timer = node.create_timer(1 / controller_frequency, track)


executor.spin()

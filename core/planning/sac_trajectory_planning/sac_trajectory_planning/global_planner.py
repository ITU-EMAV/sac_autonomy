import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler

from .hybrid_a_star import HybridAStarPlanner

from .state_node import StateNode
import torch

import torch.nn.functional as F


planner = HybridAStarPlanner(hybrid=False)

rclpy.init()
node = Node(node_name="planner_node")
executor = MultiThreadedExecutor()
executor.add_node(node)
callback_group = ReentrantCallbackGroup()

odom: Odometry = None
goal: PoseStamped = None
grid: OccupancyGrid = None

path_publisher = node.create_publisher(Path, "/planner/global_plan", 10)
marker_publisher = node.create_publisher(Marker, "/planner/global_plan_marker",10)



def plan():
    node.get_logger().info("Planning start")
    t1 = time.time()

    # check if grid exist
    if odom == None or goal == None:
        node.get_logger().info("waiting for the grid or pose")
        return

    global weights_pool
    global weights
    
    bound = 1000
    og = np.zeros((bound,bound))
    res = 0.05
    origin = np.array([-bound*res/2,-bound*res/2,0])



    # prepare positions
    start_x = odom.pose.pose.position.x
    start_y = odom.pose.pose.position.y
    start_yaw = odom.pose.pose.orientation.z
    goal_x = goal.pose.position.x
    goal_y = goal.pose.position.y
    goal_yaw = goal.pose.orientation.z

    goal_x_ = int((goal_x - origin[0]) / res)
    goal_y_ = int((goal_y - origin[1]) / res)

    # check goal is in bounds
    if goal_x_ < 0 or goal_x_ >= og.shape[0] or goal_y_ < 0 or goal_y_ >= og.shape[1]:
        node.get_logger().info("Goal is out of bounds")
        return

    # print(f"Start: {start_x}, {start_y}")
    # print(f"Goal: {goal_x}, {goal_y}")
    # HybridAstar
    x, y, yaw, v, delta, closed_set = planner.plan(
        og,
        origin,
        res,
        StateNode(start_x, start_y, start_yaw, 0.0, 0.0, 0.0, 0.0, None),
        StateNode(goal_x, goal_y, goal_yaw, 0.0, 0.0, 0.0, 0.0, None)
    )
    
    publish_path(x,y,yaw)
    # node.get_logger().warn(f"Planning took {time.time()-t1} seconds")
    
    # m = Marker()
    # m.header.frame_id = "odom"
    # m.type = Marker.POINTS
    # m.ns = "basic_shapes"
    # m.id = 0
    
    # m.scale.x = .05
    # m.scale.y = .05
    # m.scale.z = .05
    # m.color.r = 1.
    # m.color.g = 0.
    # m.color.b = 0.    
    # m.color.a = 1.
    
    # points = []
    # while not len(closed_set)==0:
    #     csx,csy = closed_set.pop()
    #     point = Point()
    #     point.x = csx
    #     point.y = csy
    #     points.append(point)
    # m.points = points
    
    # marker_publisher.publish(m)
    
    


def goal_pose_callback(msg):
    global goal
    goal = msg
    node.get_logger().info("Goal received")
    plan()


def occupancy_grid_callback(msg):
    
    
    global grid

    grid = msg


    # node.get_logger().info("occupancy grid received, sleeping 1s")


def publish_path(x,y,yaw):
    path_msg = Path()
    path_msg.header.frame_id = "odom"
    path_msg.header.stamp = node.get_clock().now().to_msg()
    for i in range(len(x)):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = x[i]
        pose.pose.position.y = y[i]
        q = quaternion_from_euler(0, 0, yaw[i])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path_msg.poses.append(pose)
    path_publisher.publish(path_msg)

def odom_callback(msg):
    global odom
    odom = msg 
    
def compute_path_callback():
    plan()
    



node.create_subscription(
    PoseStamped, "goal_pose", goal_pose_callback, 10, callback_group=callback_group
)
node.create_subscription(
    OccupancyGrid,
    "/map",
    occupancy_grid_callback,
    10,
    callback_group=callback_group,
)

node.create_timer(5,compute_path_callback)

node.create_subscription(
    Odometry,
    "/odom",
    odom_callback,
    10,
    callback_group=callback_group,
)


executor.spin()

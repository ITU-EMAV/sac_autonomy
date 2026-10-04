import time
import numpy as np
import rclpy
import math
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point, TransformStamped
from visualization_msgs.msg import Marker
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from .hybrid_a_star import HybridAStarPlanner
from .state_node import StateNode
from sac_utils.rotation_utils import transform_pose, transform_path



class LocalPlanner(Node):
    def __init__(self):
        super().__init__("planner_node")

        self.planner = HybridAStarPlanner()
        self.no_grid_map = False
        callback_group = ReentrantCallbackGroup()
        self.transform: TransformStamped = None
        self.goal: Path = None
        self.grid: OccupancyGrid = None
        self.path_publisher = self.create_publisher(
            Path, "/planner/local_plan", 10)
        self.marker_publisher = self.create_publisher(
            Marker, "/planner/local_plan_marker", 10)

        self.create_subscription(
            Path, "/planner/global_plan", self.global_plan_callback, 10, callback_group=callback_group
        )
        self.create_subscription(
            OccupancyGrid,
            "occupancy_grid",
            self.occupancy_grid_callback,
            10,
            callback_group=callback_group,
        )
        
        self.create_timer(0.2,self.compute_path_callback)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform(self, frame_from: str, frame_to: str) -> TransformStamped:
        try:
            t = self.tf_buffer.lookup_transform(
                frame_to,
                frame_from,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform between map and base_link"
            )
            return

        return t
    
    
    def compute_path_callback(self):
        self.plan()
        
    def plan(self):
        
        t1 = time.time()

        # check if grid exist
        if self.grid == None and self.no_grid_map == False :
            self.get_logger().info("waiting for the grid ")
            return
        if(self.goal == None):
            # self.get_logger().info("waiting for the goal")
            return
        
        # get the robot's position as transform msg
        robot = self.get_transform("odom", "base_link")

        # search for goal point
        min_pose_index = len(self.goal.poses)
        min_dist = 100000
        for i, p in enumerate(self.goal.poses):
            dist = (robot.transform.translation.x - p.pose.position.x) ** 2 + \
                (robot.transform.translation.y - p.pose.position.y) ** 2
            if dist < min_dist:
                min_pose_index = i
                min_dist = dist

        tracked_path = Path()

        if(len(self.goal.poses) > min_pose_index+50):
            tracked_path.poses = self.goal.poses[min_pose_index:min_pose_index+50]
            
        else:
            tracked_path.poses =  self.goal.poses
        
        robot2map = self.get_transform("odom","base_link")
        tracked_path = transform_path(tracked_path,robot2map)
        # print(len(self.goal.poses))
        # print(min_pose_index)
        # print(len(tracked_path.poses))
        # print()

        # print(time.time() - t1)

        if(self.no_grid_map == False):
            og = np.array(self.grid.data).reshape(
                self.grid.info.height,
                self.grid.info.width,
            )
            res = self.grid.info.resolution
            origin = np.array(
                [
                    self.grid.info.origin.position.x,
                    self.grid.info.origin.position.y,
                    0,
                ]
            )
        else:
            og = np.zeros((600,600))
            res = 0.05
            origin =np.array([-30/2,-30/2,0])


        # print(time.time() - t1)

        goal_y = 0
        goal_x = 0
        flag = 0
        # prepare positions

        for i in tracked_path.poses:

            x = i.pose.position.x
            y = i.pose.position.y

            

            if math.sqrt(x*x + y*y) <= 5:
                
                grid_x = round((x - origin[0]) / res)
                grid_y = round((y - origin[1]) / res)
                # print(og[grid_y][grid_x])

                if (grid_x >= 0 and grid_x < og.shape[0] and grid_y >= 0 and grid_y < og.shape[1]):
                    if(og[grid_y][grid_x] < 100):
                        # print("asdasd")
                        goal_x = x
                        goal_y = y
                        goal_yaw = i.pose.orientation.z
                else:
                    flag = 1

            if flag == 1:
                break
                
        start_x = 0.0
        start_y = 0.0
        start_yaw = 0.0
        

        # check goal is in bounds
        if goal_y == 0 and goal_x == 0:
            self.get_logger().info("Goal is out of bounds")
            return

        # print(f"Start: {start_x}, {start_y}")
        # print(f"Goal: {goal_x}, {goal_y}")
        # HybridAstar
        x, y, yaw, v, delta, closed_set = self.planner.plan(
            og,
            origin,
            res,
            StateNode(start_x, start_y, start_yaw, 0.0, 0.0, 0.0, 0.0, None),
            StateNode(goal_x, goal_y, goal_yaw, 0.0, 0.0, 0.0, 0.0, None)
        )

        self.publish_path(x, y, yaw)
        self.get_logger().warn(f"Planning took {time.time()-t1} seconds")

        # m = Marker()origin

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

    def global_plan_callback(self, msg):
        self.goal = msg
        self.get_logger().info("Goal received")
        self.plan()

    def occupancy_grid_callback(self, msg):
        self.grid = msg
        # node.get_logger().info("occupancy grid received, sleeping 1s")

    def publish_path(self, x, y, yaw):
        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(x)):
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = x[i]
            pose.pose.position.y = y[i]
            q = quaternion_from_euler(0, 0, yaw[i])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path_msg.poses.append(pose)
        self.path_publisher.publish(path_msg)


def main():
    rclpy.init()
    node = LocalPlanner()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()

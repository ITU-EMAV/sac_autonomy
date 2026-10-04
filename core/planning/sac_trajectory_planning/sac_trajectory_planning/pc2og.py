import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from geometry_msgs.msg import Pose
from sensor_msgs_py.point_cloud2 import read_points_numpy
import numpy as np
import matplotlib.pyplot as plt
import cv2
import threading
import time
import array


class PointCloudToOccupancyGrid(Node):

    def __init__(self):
        super().__init__("pc2og")


        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1  # Queue size of 1
        )
        # self.subscription = self.create_subscription(
        #     PointCloud2,
        #     "/oakd/rgb/depth/points",
        #     lambda msg: self.pointcloud_callback(msg, 0),
        #     qos_profile=qos_profile,
        #     callback_group=self.callback_0,
        # )

        self.subscription = self.create_subscription(
            PointCloud2,
            "/zed/pointcloud_0",
            lambda msg: self.pointcloud_callback(msg, 1),
            qos_profile = qos_profile,
            callback_group = ReentrantCallbackGroup()
        )

        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid, "/occupancy_grid", 10
        )


        # width, height, resolution in meters
        self.resolution = 0.05
        self.height = 30
        self.width = 30


        self.og_msg = OccupancyGrid()
        self.og_msg.info.height = int(self.height / self.resolution)
        self.og_msg.info.width = int(self.width / self.resolution)
        self.og_msg.info.resolution = self.resolution
        self.og_msg.header.frame_id = "base_link"
        self.og_msg.info.origin.position.x = -self.height / 2
        self.og_msg.info.origin.position.y = -self.width / 2
        self.og = np.full(
            (self.og_msg.info.height, self.og_msg.info.width), -1, dtype=np.int8
        )

        self.is_plotting = False
        self.expand_occupied_cells = True
        self.i = 0

                    # Create a structuring element for dilation (a square with side length corresponding to the radius)
        kernel_size = int(1.0 / self.resolution)
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_size, kernel_size)
        )

        print("PCL to OG working..")

    


    def pointcloud_callback(self, msg: PointCloud2, idx):

        self.extract_xyz_from_pointcloud2(msg)
        # process_thread = threading.Thread(
        #     target=self.process_pointcloud_thread, 
        #     args=(self.pcl,)
        # )
        # process_thread.start()
        self.process_pointcloud_thread(self.pcl)

    def process_pointcloud_thread(self,pcl):
        # print(self.i)
        # self.i = self.i + 1

        # t = time.time()
        # Use a single operation to filter NaNs and Infs
        mask = ~np.isnan(pcl).any(axis=1) & ~np.isinf(pcl).any(axis=1)
        pcl = pcl[mask]

        idx = 1
        if idx == 0:  # if it is simulation
            pcl[:, 0] = pcl[:, 0] + 1.0
            pcl[:, 2] = pcl[:, 2] + 1.6

        # filter out the ground
        pcl = pcl[pcl[:, 2] > 0.1]

        # TODO: implement raycast here

        # move the origin to the center of the occupancy grid
        pcl[:, 0] = pcl[:, 0] - self.og_msg.info.origin.position.x
        pcl[:, 1] = pcl[:, 1] - self.og_msg.info.origin.position.y

        pcl = np.fix(pcl / self.resolution)

        # print("t1 : ",(time.time()-t)*1000000)
        # t = time.time()

        if self.is_plotting:
            plt.scatter(pcl[:, 0], pcl[:, 1], color="blue", marker="o")

            # Set labels and title
            plt.xlabel("X coordinates")
            plt.ylabel("Y coordinates")
            plt.title("2D Points")

            # Display the plot
            plt.grid(True)
            plt.show()

        # mark these points as occupied in the occupancy grid

        self.og = np.full(
            (self.og_msg.info.height, self.og_msg.info.width), 0, dtype=np.int8
        )


        x_indices = pcl[:, 0].astype(np.int32)
        y_indices = pcl[:, 1].astype(np.int32)
        self.og[y_indices, x_indices] = 100

        # print("t2 : ",(time.time()-t)*1000000)
        # t = time.time()
        

        if self.expand_occupied_cells:

            occupied = (self.og == 100).astype(
                np.uint8
            )  # Assuming 100 represents occupied cells



            # Perform dilation to expand the occupied cells
            expanded_occupied = cv2.dilate(occupied, self.kernel, iterations=1)

            # Convert the expanded occupancy grid back to the original data format
            expanded_data = np.where(expanded_occupied == 1, 100, 0).astype(np.int8)

            # Create a new OccupancyGrid message with the expanded data
            self.og_msg.data = array.array('b', expanded_data.astype(np.int8, copy=False).ravel())
        else:
            self.og_msg.data = array.array('b', self.og.astype(np.int8, copy=False).ravel())
        # print("expand : ",(time.time()-t)*1000000)
        # t = time.time()

        self.og_msg.header.stamp = self.get_clock().now().to_msg()
        self.occupancy_grid_pub.publish(self.og_msg)
        # print("publish : ",(time.time()-t)*1000000)
        # t = time.time()
        # print()
        # print()

    def extract_xyz_from_pointcloud2(self, msg: PointCloud2):
        # points = torch.frombuffer(msg.data,dtype= torch.float32).reshape(-1, msg.point_step // 4)
        self.pcl = read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        # print(f"points shape: {points.shape}")
        

        # dtype = np.dtype(
        #     [
        #         ("x", np.float32),
        #         ("y", np.float32),
        #         ("z", np.float32),
        #         # ("void", np.float32),
        #         # ("rgb", np.float32),
        #     ]
        # )

        # # print the message fields
        # print(f"fields: {msg.fields}")
        # print(f"point step: {msg.point_step}")

        # points = np.frombuffer(
        #     msg.data, dtype=dtype, count=(len(msg.data) // msg.point_step)
        # )
        # points = np.array([points["x"], points["y"], points["z"]]).T

        # print(f"points shape: {points.shape}")
        # return points


def main():
    rclpy.init()

    pc2og = PointCloudToOccupancyGrid()

    executor = MultiThreadedExecutor()
    executor.add_node(pc2og)
    executor.spin()

    rclpy.shutdown()

    rclpy.init()


if __name__ == "__main__":
    main()

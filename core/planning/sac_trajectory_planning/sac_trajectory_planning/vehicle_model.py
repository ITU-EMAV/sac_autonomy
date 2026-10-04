import time
import numpy as np

pi = np.pi

from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from nav_msgs.msg import OccupancyGrid
from .state_node import StateNode


class VehicleModel:
    def __init__(self) -> None:
        self.previous_state = StateNode(0, 0, 0, 0, 0, 0, 0, None)
        # Constants
        self.MAX_STEERING_ANGLE = 0.5  # rad
        self.MAX_ACCELERATION = 1.0  # m/s^2
        self.MAX_VELOCITY = 10.0  # m/s
        self.SAMPLING_TIME = 1.0  # s
        self.L = 0.5 #2.0  # m distance between front and rear wheels

    def move(self, prev: StateNode, input_v: float, input_delta: float) -> StateNode:
        # R = self.L / np.tan(input_delta)
        x = prev.x + prev.v * np.cos(prev.yaw + input_delta) * self.SAMPLING_TIME
        y = prev.y + prev.v * np.sin(prev.yaw + input_delta) * self.SAMPLING_TIME
        yaw = self.pi_2_pi(
            prev.yaw + prev.v / self.L * np.tan(input_delta) * self.SAMPLING_TIME
        )
        v = input_v
        delta = input_delta

        return StateNode(x, y, yaw, v, delta, 0, 0, prev)

    def set_pose(self, pose: PoseStamped) -> None:
        self.current_pose = pose

    def set_twist(self, twist: TwistStamped) -> None:
        self.current_twist = twist

    def set_occupancy_grid(self, occupancy_grid: OccupancyGrid) -> None:
        self.occupancy_grid = occupancy_grid

    def pi_2_pi(self, angle):
        return (angle + pi) % (2 * pi) - pi

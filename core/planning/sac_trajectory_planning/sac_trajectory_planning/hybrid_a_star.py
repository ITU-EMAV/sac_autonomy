import time
import numpy as np
import queue
import math
from .vehicle_model import VehicleModel
from .state_node import StateNode



class HybridAStarPlanner:
    def __init__(
        self,
        grid_resolution=0.05,
        hybrid = True
    ):
        sampling_time = 0.25  # 1.0  # s
        self.vm = VehicleModel()
        self.vm.SAMPLING_TIME = sampling_time

        self.hybrid = hybrid

        self.resolution = grid_resolution
        self.occupancy_grid = None
        self.origin = None
        self.goal_pose = None
        self.path = None
        self.state = None
        self.obstacles = {}
        self.linear_velocity = 1.0  # nominal linear velocity in m/s
        self.steering_angle_resolution = 0.1  # rad
        self.steering_angle_limit = 0.5  # rad
        self.g_cost = self.linear_velocity * sampling_time
        self.motions = []
        if(hybrid):
            self.generate_motions(
                self.linear_velocity,
                self.steering_angle_resolution,
                self.steering_angle_limit,
            )
        else:
            res = grid_resolution*2
            sqrt_2 = math.sqrt(2.0*res*res)
            step = res
            self.motions = [[step,0,step],[0,step,step],[-step,0,step],[0,-step,step],[step,step,sqrt_2],[step,-step,sqrt_2],[-step,step,sqrt_2],[-step,-step,sqrt_2]]
        self.radius = int(1.0 / 0.05)  # car's radius in grid cells
        print("Path planner is ready.")

    def generate_motions(
        self, linear_velocity, steering_angle_resolution, steering_angle_limit
    ):
        s = int(steering_angle_limit / steering_angle_resolution * 2)
        for i in range(s + 1):
            self.motions.append(
                [
                    linear_velocity,
                    steering_angle_limit - i * steering_angle_resolution,
                    self.g_cost,
                ],
            )
        print("Possible velocity and steering angle pairs:")
        for i in self.motions:
            print(f"v: {i[0]}, s: {i[1]}, cost: {i[2]}")

    def plan(
        self,
        occupancy_grid: np.array,
        origin: np.array,
        grid_resolution: float,
        start: StateNode,
        end: StateNode
    ) -> list:
        self.occupancy_grid = occupancy_grid
        self.resolution = grid_resolution
        self.origin = origin
        oy, ox = np.where(occupancy_grid == 100)
        # print(f"ox: {ox}, oy: {oy}")

        x, y, yaw, v, delta = list(), list(), list(), list(), list()

        # a star path planning
        open_set = queue.PriorityQueue()
        closed_set = set()
        
        visited_set = set()

        open_set.put((0, start))

        t1 = time.time()
        max_iter = 30000
        cur_iter = 0
        while not open_set.empty():
            if cur_iter > max_iter:
                print("Max iteration reached")
                return x, y, yaw, v, delta, visited_set
            cur_iter +=1
            # get the node with the lowest f_cost
            current = open_set.get()
            cost, node = current

            # if the goal is reached, retrieve the path
            if (
                np.abs(node.x - end.x) <= self.resolution * 5
                and np.abs(node.y - end.y) <= self.resolution * 5
            ):
                # if node.x == end.x and node.y == end.y:
                # print(f"Goal reached, time {time.time() - t1}")
                # print(f"end node: {node.x}, {node.y}, {node.yaw}")
                path = self.retrieve_path(node)
                for state in path:
                    x.append(state.x)
                    y.append(state.y)
                    yaw.append(state.yaw)
                    v.append(state.v)
                    delta.append(state.delta)
                    
                
                return x, y, yaw, v, delta, visited_set

            

            
            move_states = []
            for m in self.motions:
                if(self.hybrid == True):
                    new_state = self.vm.move(node, m[0], m[1])
                else:
                    new_state = StateNode(x=node.x+m[0],y=node.y+m[1],yaw = 0,delta = 0,v = 0,parent=node) 

                # print(f"rounded new_state: {new_state.x}, {new_state.y}")

                rounded_x = self.coordinate_round(new_state.x, self.resolution)
                rounded_y = self.coordinate_round(new_state.y, self.resolution)
            
                # rounded_yaw = self.yaw_round(new_state.yaw)
                # print(f"rounded new_state: {rounded_x}, {rounded_y}")
                # check if the new state is occupied
                grid_x, grid_y = self.state2grid(new_state)

                

                if (rounded_x, rounded_y) in closed_set:
                    continue

                
                if grid_x>= occupancy_grid.shape[0] or grid_y >= occupancy_grid.shape[1] or occupancy_grid[grid_y, grid_x] >= 100 or occupancy_grid[grid_y, grid_x] <= -1 :
                    # print("\n\n\nOccupied")
                    continue
                closed_set.add((rounded_x, rounded_y))

                grid_cost = occupancy_grid[grid_y, grid_x]/20
                
                
                visited_set.add((new_state.x, new_state.y))

                # calculate the cost
                new_state.g_cost = node.g_cost + m[2] 

                new_state.h_cost = (end.x - new_state.x) ** 2 + (end.y - new_state.y) ** 2 # manhattan
                
                new_cost = new_state.g_cost + new_state.h_cost + grid_cost
                # print(
                #     f"x: {new_state.x}, y: {new_state.y},yaw {new_state.yaw}, cost: {new_cost}"
                # )

                open_set.put((new_cost, new_state))
                

        # path not found, return empty list
        print("Path not found")
        return x, y, yaw, v, delta, visited_set

    def state2grid(self, state: StateNode) -> tuple:
        return (
            int((state.x - self.origin[0]) / self.resolution),
            int((state.y - self.origin[1]) / self.resolution),
        )

    def coordinate_round(self, value, resolution):
        return round(value / resolution)

    def yaw_round(self, angle):
        return int(self.vm.pi_2_pi(angle) / (np.pi / 4 / 10))

    def retrieve_path(self, current: StateNode) -> list:
        path = []
        while current is not None:
            path.append(current)
            current = current.parent
        path = path[::-1]
        # print(path)
        return path

    def expansion(self):
        self.vm.move()

import numpy as np
import math

# -----------------------------
# Pure Pursuit Controller Class
# -----------------------------
class PurePursuitController:
    def __init__(self, wheel_base=1.8, look_ahead_gain=0.1, look_ahead_distance=2.0):
        self.wheel_base = wheel_base
        self.look_ahead_gain = look_ahead_gain
        self.look_ahead_distance = look_ahead_distance
        self.old_nearest_index = None
        self.previous_index = 0

    def search_target_index(self, vehicle_state, path_x, path_y):
        """ Finds the look-ahead target index on the path. """
        if self.old_nearest_index is None:
            # First search for closest point
            distances = np.hypot(np.array(path_x) - vehicle_state.rear_x, np.array(path_y) - vehicle_state.rear_y)
            index = np.argmin(distances)
            self.old_nearest_index = index
        else:
            index = self.old_nearest_index
            while index < len(path_x) - 1:
                if vehicle_state.calc_distance(path_x[index + 1], path_y[index + 1]) > \
                        vehicle_state.calc_distance(path_x[index], path_y[index]):
                    break
                index += 1
            self.old_nearest_index = index

        # Compute dynamic look-ahead distance
        look_ahead_distance = self.look_ahead_gain * vehicle_state.v + self.look_ahead_distance

        # Find the target point
        while (index < len(path_x) - 1) and (look_ahead_distance > vehicle_state.calc_distance(path_x[index], path_y[index])):
            index += 1

        return index, look_ahead_distance

    def compute_steering_angle(self, vehicle_state, path_x, path_y):
        """ Compute the steering angle using Pure Pursuit Algorithm. """
        index, look_ahead_distance = self.search_target_index(vehicle_state, path_x, path_y)

        if self.previous_index >= index:
            index = self.previous_index

        if index < len(path_x):
            tx, ty = path_x[index], path_y[index]
        else:
            tx, ty = path_x[-1], path_y[-1]
            index = len(path_x) - 1
        
        self.previous_index = index
        # Compute angle to the target
        alpha = math.atan2(ty - vehicle_state.rear_y, tx - vehicle_state.rear_x) - vehicle_state.yaw

        # Compute steering angle
        steering_angle = math.atan2(2.0 * self.wheel_base * math.sin(alpha) / look_ahead_distance, 1.0)

        return steering_angle

# -----------------------------
# Stanley Controller Class
# -----------------------------
class StanleyController:
    def __init__(self, wheel_base=1.8, control_gain=0.01, max_steering_angle=np.radians(20.0)):
        self.wheel_base = wheel_base
        self.control_gain = control_gain  # Stanley gain (k)
        self.max_steering_angle = max_steering_angle
        self.previous_index = 0  # Store previous target index

    def search_target_index(self, vehicle_state, path_x, path_y):
        """ Finds the closest index on the path and computes the cross-track error. """
        # Compute front axle position
        fx = vehicle_state.x + self.wheel_base * np.cos(vehicle_state.yaw)
        fy = vehicle_state.y + self.wheel_base * np.sin(vehicle_state.yaw)

        # Compute distances to path points
        dx = np.array(path_x) - fx
        dy = np.array(path_y) - fy
        distances = np.hypot(dx, dy)

        # Find the closest point
        index = np.argmin(distances)

        # Compute cross-track error
        front_axle_vector = [-np.cos(vehicle_state.yaw + np.pi / 2), -np.sin(vehicle_state.yaw + np.pi / 2)]
        cross_track_error = np.dot([dx[index], dy[index]], front_axle_vector)

        return index, cross_track_error

    def compute_steering_angle(self, vehicle_state, path_x, path_y):
        """ Compute the steering angle using the Stanley Control Algorithm. """
        index, cross_track_error = self.search_target_index(vehicle_state, path_x, path_y)

        if self.previous_index >= index:
            index = self.previous_index

        self.previous_index = index

        path_yaw = np.arctan2(path_y[1:] - path_y[0:-1] , path_x[1:] - path_x[0:-1])
        # Compute heading error
        path_heading = path_yaw[index-1]
        heading_error = self.normalize_angle(path_heading - vehicle_state.yaw)

        # Compute cross-track correction
        steering_correction = np.arctan2(self.control_gain * cross_track_error, vehicle_state.v + 1e-5)

        # Compute final steering angle
        steering_angle = heading_error + steering_correction
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        return steering_angle

    @staticmethod
    def normalize_angle(angle):
        """ Normalize angle to [-pi, pi]. """
        return (angle + np.pi) % (2 * np.pi) - np.pi

# -----------------------------
# Vehicle State Class
# -----------------------------
class VehicleState:
    def __init__(self,  x=0.0, y=0.0, yaw=0.0, v=0.0, dt=0.1, wheel_base=1.8):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.wheel_base = wheel_base
        self.rear_x = self.x - ((self.wheel_base / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.wheel_base / 2) * math.sin(self.yaw))
        self.dt = dt

    def update(self, acceleration, steering_angle):
        """ Update the vehicle state using the bicycle model. """
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += (self.v / self.wheel_base) * math.tan(steering_angle) * self.dt
        self.v += acceleration * self.dt
        self.rear_x = self.x - ((self.wheel_base / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.wheel_base / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        """ Calculate the Euclidean distance from the vehicle to a given point. """
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)
# -----------------------------
# Run Simulation as test
# -----------------------------
if __name__ == '__main__':

    """ Runs a simulation using the Pure Pursuit controller. """
    
    print("Starting Pure Pursuit Simulation...")
    import matplotlib.pyplot as plt
    
    # -----------------------------
    # Parameters
    # -----------------------------
    k = 0.1  # Look-ahead gain
    Lfc = 2.0  # [m] Look-ahead distance
    Kp = 1.0  # Speed proportional gain
    dt = 0.1  # [s] Time tick
    WB = 1.8  # [m] Wheelbase of vehicle
    
    # Generate example path
    cx = np.arange(0, 100, 0.5)
    cy = np.array([math.sin(ix / 5.0) * ix / 2.0 for ix in cx])

    # cx = 10*np.cos(np.arange(0, 50, 0.5)/5)
    # cy = 10*np.sin(np.arange(0, 50, 0.5)/5)
    

    target_speed = 5.0  # [m/s]
    max_simulation_time = 100.0  # [s]

    # Initialize vehicle state
    vehicle_state = VehicleState(x=-0.0, y=-3.0, yaw=0.0, v=0.0, wheel_base = WB)
    controller = PurePursuitController(wheel_base=WB, look_ahead_gain=k, look_ahead_distance=Lfc)
    
    # Simulation parameters
    time = 0.0
    last_index = len(cx) - 1
    target_index, _ = controller.search_target_index(vehicle_state, cx, cy)
    trajectory_x, trajectory_y = [], []

    while max_simulation_time >= time and last_index > controller.previous_index:
        # Compute control inputs
        current_speed = vehicle_state.v
        acceleration = Kp * (target_speed - current_speed)
        steering_angle = controller.compute_steering_angle(vehicle_state, cx, cy) 

        # Update vehicle state
        vehicle_state.update(acceleration, steering_angle)

        # Store trajectory
        trajectory_x.append(vehicle_state.x)
        trajectory_y.append(vehicle_state.y)

        # Time update
        time += dt

    # Plot Results
    plt.figure(figsize=(10, 6))
    plt.plot(cx, cy, ".r", label="Reference Path")
    plt.plot(trajectory_x, trajectory_y, "-b", label="Vehicle Trajectory")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.title("Pure Pursuit Path Tracking")
    plt.show()


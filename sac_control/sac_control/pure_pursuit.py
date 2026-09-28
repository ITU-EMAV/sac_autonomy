"""Pure pursuit: drives the car along the planned path.

Steering: the classic pure pursuit on the rear axle. A point on the path one lookahead
distance ahead is chosen and the car drives the circle arc through it. The lookahead grows
with speed, which keeps fast driving steady.

Speed: the path's curvature limits the speed (lateral acceleration), braking starts early
enough to reach each corner's speed (and a stop at the end of an open path), and the
command changes no faster than the acceleration limits.

Safety: when the car is further than `max_off_path` from the path it stops, and stays
stopped until a path is published again.

Subscribes:  path     nav_msgs/Path in the map frame (remapped to /sac/planning/path); a loop
                      (last pose next to the first) is driven round and round
             TF       map -> base_footprint, from the localization (or the simulation)
Publishes:   cmd_vel  geometry_msgs/Twist, speed and yaw rate (remapped to
                      /sac/actuators/cmd_vel), only while it has a path and a pose
             ~/lookahead          geometry_msgs/PointStamped, the point it steers to
             ~/cross_track_error  std_msgs/Float64 [m], positive left of the path
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


class PurePursuit(Node):
    def __init__(self):
        super().__init__("pure_pursuit")
        p = self.declare_parameter
        self.wheel_base = p("wheel_base", 1.873).value  # sac_description
        self.max_steering = p("max_steering", 0.6).value  # [rad]
        self.max_speed = p("max_speed", 10.0).value  # [m/s]
        self.max_lateral_acceleration = p("max_lateral_acceleration", 3.0).value  # [m/s^2]
        self.max_acceleration = p("max_acceleration", 2.0).value  # [m/s^2]
        self.max_deceleration = p("max_deceleration", 4.0).value  # [m/s^2]
        # Lookahead = min + gain * speed, at most max [m]
        self.lookahead_min = p("lookahead_min", 4.0).value
        self.lookahead_gain = p("lookahead_gain", 0.6).value  # [s]
        self.lookahead_max = p("lookahead_max", 20.0).value
        # Stop when the car is further than this from the path [m]
        self.max_off_path = p("max_off_path", 4.0).value
        self.map_frame = p("map_frame", "map").value
        self.robot_frame = p("robot_frame", "base_footprint").value
        rate = p("rate", 20.0).value  # [Hz]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "path", self.on_path, latched)
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.lookahead_publisher = self.create_publisher(PointStamped, "~/lookahead", 10)
        self.error_publisher = self.create_publisher(Float64, "~/cross_track_error", 10)

        self.path = None  # (n, 2) points
        self.speed = 0.0  # last commanded speed
        self.index = None  # nearest path point, tracked along the path
        self.driving = False
        self.finished = False
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self.tick)
        # Limits and lookahead can be changed while driving (ros2 param set)
        self.add_post_set_parameters_callback(self.on_parameters)

    def on_parameters(self, parameters):
        tunable = {
            "max_speed", "max_lateral_acceleration", "max_acceleration", "max_deceleration",
            "lookahead_min", "lookahead_gain", "lookahead_max", "max_off_path",
        }
        for parameter in parameters:
            if parameter.name in tunable:
                setattr(self, parameter.name, float(parameter.value))
        if self.path is not None:
            self.speed_limit = self.speed_profile(self.path, self.spacing, self.closed)

    # ---------------------------------------------------------------- path
    def on_path(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().error(f"Path in '{msg.header.frame_id}', expected '{self.map_frame}'")
            return
        points = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        if len(points) < 3:
            self.get_logger().warn("Path with fewer than 3 poses, ignored")
            return
        steps = np.hypot(*np.diff(points, axis=0).T)
        spacing = float(np.median(steps))
        self.closed = bool(np.hypot(*(points[-1] - points[0])) < 3 * spacing)
        self.path = points
        self.spacing = spacing
        self.speed_limit = self.speed_profile(points, spacing, self.closed)
        self.index = None
        self.finished = False
        self.get_logger().info(
            f"Path: {len(points)} poses, {len(points) * spacing:.0f} m, "
            f"{'loop' if self.closed else 'open'}, top speed on it "
            f"{self.speed_limit.max():.1f} m/s"
        )

    def speed_profile(self, points, spacing, closed):
        """Highest speed at each path point: corners (lateral acceleration) and braking
        towards them."""
        n = len(points)
        k = max(1, int(round(3.0 / spacing)))  # curvature over ~6 m
        idx = np.arange(n)
        if closed:
            a, b, c = points[(idx - k) % n], points, points[(idx + k) % n]
        else:
            a, b, c = points[np.clip(idx - k, 0, n - 1)], points, points[np.clip(idx + k, 0, n - 1)]
        # Circle through three points: curvature = 4 * area / (|ab| |bc| |ca|)
        cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
        lengths = (
            np.hypot(*(b - a).T) * np.hypot(*(c - b).T) * np.hypot(*(a - c).T)
        )
        curvature = np.abs(2 * cross) / np.maximum(lengths, 1e-9)
        limit = np.minimum(self.max_speed, np.sqrt(self.max_lateral_acceleration / np.maximum(curvature, 1e-6)))
        if not closed:
            limit[-1] = 0.0
        # Backwards: the speed from which the car can still brake to the next limit
        for _ in range(2 if closed else 1):
            for i in range(n - 2 if not closed else n - 1, -1, -1):
                following = limit[(i + 1) % n]
                limit[i] = min(limit[i], math.sqrt(following**2 + 2 * self.max_deceleration * spacing))
        return limit

    # ---------------------------------------------------------------- control
    def tick(self):
        if self.path is None or self.finished:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time(), timeout=Duration(seconds=0.0)
            ).transform
        except TransformException as error:
            self.get_logger().warn(f"No pose: {error}", throttle_duration_sec=2.0)
            self.stop()
            return
        q = t.rotation  # yaw of the full orientation: the track has hills
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        # Pure pursuit steers the rear axle
        rear = np.array([t.translation.x, t.translation.y]) - heading * self.wheel_base / 2

        i = self.nearest(rear)
        off_path = float(np.hypot(*(self.path[i] - rear)))
        if off_path > self.max_off_path:
            self.finished = True
            self.stop()
            self.get_logger().error(
                f"Stopped: {off_path:.1f} m from the path (limit {self.max_off_path} m). "
                "Publish the path again to restart."
            )
            return
        remaining = None if self.closed else (len(self.path) - 1 - i) * self.spacing
        if remaining is not None and remaining < 0.5:
            self.finished = True
            self.stop()
            self.get_logger().info("End of the path")
            return

        # Speed: the profile a little ahead (the car needs time to slow down)
        preview = int(round(max(1.0, self.speed * 0.5) / self.spacing))
        target = float(self.speed_limit[self.wrap(i + preview)])
        limit_up = self.speed + self.max_acceleration * self.dt
        limit_down = self.speed - self.max_deceleration * self.dt
        self.speed = min(max(target, limit_down), limit_up)

        # Steering: the circle through the lookahead point
        lookahead = min(self.lookahead_max, self.lookahead_min + self.lookahead_gain * self.speed)
        goal = self.lookahead_point(rear, i, lookahead)
        d = goal - rear
        x = d @ heading
        y = -d[0] * heading[1] + d[1] * heading[0]
        curvature = 2 * y / max(x * x + y * y, 1e-6)
        max_curvature = math.tan(self.max_steering) / self.wheel_base
        curvature = min(max(curvature, -max_curvature), max_curvature)

        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = self.speed * curvature
        self.cmd_publisher.publish(cmd)
        self.driving = True
        self.publish_debug(goal, rear, i)

    def wrap(self, i):
        n = len(self.path)
        return i % n if self.closed else min(i, n - 1)

    def nearest(self, point):
        """Nearest path point; after the first search only a window ahead of the last one,
        so a path that crosses itself or passes close to itself is followed in order."""
        n = len(self.path)
        if self.index is None:
            candidates = np.arange(n)
        else:
            window = int(20.0 / self.spacing)
            candidates = np.array([self.wrap(self.index + j) for j in range(-2, window)])
        distances = np.hypot(*(self.path[candidates] - point).T)
        self.index = int(candidates[np.argmin(distances)])
        return self.index

    def lookahead_point(self, rear, i, lookahead):
        """First path point at least `lookahead` from the rear axle, going forwards."""
        n = len(self.path)
        for j in range(n if self.closed else n - i):
            p = self.path[self.wrap(i + j)]
            if np.hypot(*(p - rear)) >= lookahead:
                return p
        return self.path[self.wrap(i + n)]

    def publish_debug(self, goal, rear, i):
        point = PointStamped()
        point.header.frame_id = self.map_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x, point.point.y = float(goal[0]), float(goal[1])
        self.lookahead_publisher.publish(point)

        a = self.path[i]
        b = self.path[self.wrap(i + 1)]
        tangent = (b - a) / max(np.hypot(*(b - a)), 1e-9)
        offset = rear - a
        self.error_publisher.publish(Float64(data=float(tangent[0] * offset[1] - tangent[1] * offset[0])))

    def stop(self):
        self.speed = 0.0
        if self.driving:
            self.cmd_publisher.publish(Twist())
            self.driving = False


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

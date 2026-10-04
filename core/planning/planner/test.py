#!/usr/bin/env python3
"""Pure pursuit kontrolcusunun /cmd_vel ciktisini canli cizen basit monitor.

controller_exe artik geometry_msgs/Twist yayinliyor:
    linear.x  -> komut hizi v [m/s]
    angular.z -> direksiyon acisi delta [rad]
"""
import math
import threading
import time
import tkinter as tk
from collections import deque

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Twist
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from rclpy.node import Node


class CmdVelPlotter(Node):
    def __init__(self, master):
        super().__init__('cmd_vel_plotter')

        self.master = master
        self.master.title("cmd_vel Monitor")

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        topic_name = self.get_parameter('cmd_vel_topic').value

        self.subscription = self.create_subscription(
            Twist,
            topic_name,
            self.listener_callback,
            10
        )

        self.max_points = 300
        self.speed_data = deque(maxlen=self.max_points)
        self.steer_data = deque(maxlen=self.max_points)
        self.time_data = deque(maxlen=self.max_points)
        self.start_time = time.time()

        # Create matplotlib figure: hiz ve direksiyon acisi ayri eksenlerde
        self.fig, (self.ax_v, self.ax_d) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        self.fig.tight_layout(pad=3.0)

        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Update plot periodically
        self.update_plot()

        self.get_logger().info(
            f"Subscribed to {topic_name}, plotting v & steering in real time...")

    def listener_callback(self, msg: Twist):
        current_time = time.time() - self.start_time
        try:
            v = float(msg.linear.x)                       # m/s
            delta = math.degrees(float(msg.angular.z))    # rad -> deg

            self.time_data.append(current_time)
            self.speed_data.append(v)
            self.steer_data.append(delta)
            self.get_logger().info(f"v: {v:.2f} m/s, delta: {delta:.2f} deg")

        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")

    def update_plot(self):
        if len(self.time_data) >= 2:
            try:
                # Create fresh lists - avoid any numpy conversion
                x_data = [float(t) for t in self.time_data]
                v_data = [float(v) for v in self.speed_data]
                d_data = [float(d) for d in self.steer_data]

                x_min, x_max = min(x_data), max(x_data)

                # Clear and replot instead of updating
                self.ax_v.clear()
                self.ax_v.plot(x_data, v_data, 'r-', linewidth=2, label='v (m/s)')
                self.ax_v.set_ylabel('Linear velocity (m/s)', fontsize=11)
                self.ax_v.set_title('cmd_vel Over Time', fontsize=14)

                self.ax_d.clear()
                self.ax_d.plot(x_data, d_data, 'b-', linewidth=2, label='delta (deg)')
                self.ax_d.axhline(0.0, color='gray', linewidth=0.8, alpha=0.5)
                self.ax_d.set_ylabel('Steering angle (deg)', fontsize=11)
                self.ax_d.set_xlabel('Time (s)', fontsize=11)

                for ax, data in ((self.ax_v, v_data), (self.ax_d, d_data)):
                    lo, hi = min(data), max(data)
                    pad = max(0.1, 0.1 * (hi - lo))
                    ax.set_xlim(x_min - 0.5, x_max + 0.5)
                    ax.set_ylim(lo - pad, hi + pad)
                    ax.grid(True, alpha=0.3)
                    ax.legend(loc='upper right')

                self.canvas.draw()

            except Exception as e:
                self.get_logger().error(f"Plotting error: {e}")

        # Schedule next update
        self.master.after(50, self.update_plot)


def spin_ros(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)

    root = tk.Tk()
    node = CmdVelPlotter(root)

    # Spin ROS in background
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

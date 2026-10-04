#!/usr/bin/env python3
"""Dry-run capable target-speed to pedal controller, upstream of Guardian."""
import math
import time
import copy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sac_interfaces.msg import ActuatorCommand, VehicleSpeed


class SpeedLoop:
    def __init__(self, kp, ki, brake_kp, max_throttle, max_brake, max_accel,
                 max_decel, stop_brake, integral_limit):
        self.kp, self.ki, self.brake_kp = kp, ki, brake_kp
        self.max_throttle, self.max_brake = max_throttle, max_brake
        self.max_accel, self.max_decel = max_accel, max_decel
        self.stop_brake, self.integral_limit = stop_brake, integral_limit
        self.integral = 0.0
        self.reference = 0.0

    def update(self, target, measured, dt):
        if not all(math.isfinite(v) for v in (target, measured, dt)) or dt <= 0:
            self.integral = 0.0
            return 0.0, self.max_brake
        target = max(0.0, target)
        self.reference = min(max(target, self.reference-self.max_decel*dt),
                             self.reference+self.max_accel*dt)
        if target <= 0.01:
            self.integral = 0.0
            return 0.0, self.stop_brake
        error = self.reference-measured
        if error < -0.03 or target <= 0.01:
            self.integral = 0.0
            return 0.0, min(self.max_brake, max(0.0, -error*self.brake_kp))
        raw = self.kp*error+self.ki*self.integral
        if 0.0 < raw < self.max_throttle or (raw >= self.max_throttle and error < 0):
            self.integral = max(-self.integral_limit,
                                min(self.integral_limit, self.integral+error*dt))
            raw = self.kp*error+self.ki*self.integral
        return min(self.max_throttle, max(0.0, raw)), 0.0


class LongitudinalController(Node):
    def __init__(self):
        super().__init__('longitudinal_controller')
        defaults = dict(command_topic='/control/controller_command',
                        output_topic='/control/longitudinal_command',
                        speed_topic='/vehicle/speed', command_timeout=0.12,
                        speed_timeout=0.15, kp=0.15, ki=0.05, brake_kp=0.4,
                        max_throttle=0.30, max_brake=1.0, max_accel=0.5,
                        max_decel=1.5, stop_brake=1.0, integral_limit=2.0,
                        period=0.02)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda name: self.get_parameter(name).value
        for key in defaults.keys()-{'command_topic', 'output_topic', 'speed_topic'}:
            if not math.isfinite(p(key)) or p(key) <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.loop = SpeedLoop(*(p(k) for k in (
            'kp', 'ki', 'brake_kp', 'max_throttle', 'max_brake', 'max_accel',
            'max_decel', 'stop_brake', 'integral_limit')))
        self.command = None
        self.command_received = None
        self.speed = None
        self.speed_stamp = None
        self.speed_received = None
        self.last_tick = time.monotonic()
        self.sequence = 0
        self.pub = self.create_publisher(ActuatorCommand, p('output_topic'), 1)
        self.create_subscription(ActuatorCommand, p('command_topic'), self.on_command, 1)
        self.create_subscription(VehicleSpeed, p('speed_topic'), self.on_speed, 1)
        self.create_timer(p('period'), self.tick)

    def on_command(self, msg):
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.source_stamp)
        age = (now-stamp).nanoseconds*1e-9
        if msg.source_id != 'planner_controller' or not 0 <= age <= self.get_parameter('command_timeout').value:
            return
        if not math.isfinite(msg.target_speed_mps) or msg.target_speed_mps < 0:
            return
        self.command, self.command_received = msg, time.monotonic()

    def on_speed(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        speed = msg.speed_mps
        if not math.isfinite(speed) or speed < 0 or not 0 <= age <= self.get_parameter('speed_timeout').value:
            return
        if self.speed_stamp is not None and stamp <= self.speed_stamp:
            return
        self.speed, self.speed_stamp = speed, stamp
        self.speed_received = time.monotonic()

    def tick(self):
        now = time.monotonic()
        dt = min(max(now-self.last_tick, 1e-3), 0.1)
        self.last_tick = now
        speed_ok = (self.speed_received is not None and
                    now-self.speed_received <= self.get_parameter('speed_timeout').value and
                    0 <= (self.get_clock().now()-self.speed_stamp).nanoseconds*1e-9 <=
                    self.get_parameter('speed_timeout').value)
        command_ok = (self.command_received is not None and
                      now-self.command_received <= self.get_parameter('command_timeout').value and
                      0 <= (self.get_clock().now()-Time.from_msg(self.command.source_stamp)).nanoseconds*1e-9 <=
                      self.get_parameter('command_timeout').value)
        if command_ok:
            output = copy.deepcopy(self.command)
        else:
            output = ActuatorCommand()
            output.mode = ActuatorCommand.MODE_CONTROLLED_STOP
            output.source_id = 'planner_controller'
        if speed_ok and command_ok and output.mode == ActuatorCommand.MODE_AUTONOMOUS:
            throttle, brake = self.loop.update(output.target_speed_mps, self.speed, dt)
        else:
            self.loop.integral = 0.0
            self.loop.reference = 0.0
            throttle, brake = 0.0, self.loop.max_brake
            output.mode = ActuatorCommand.MODE_CONTROLLED_STOP
            output.target_speed_mps = 0.0
        output.throttle_normalized = throttle
        output.brake_normalized = brake
        output.drive_torque_nm = 0.0
        output.armed = False
        output.enable = False
        output.source_stamp = self.get_clock().now().to_msg()
        output.publication_stamp = output.source_stamp
        output.source_steady_time_ns = time.monotonic_ns()
        self.sequence += 1
        output.sequence = self.sequence
        output.validity_duration.sec = 0
        output.validity_duration.nanosec = 100_000_000
        output.validity_flags = (ActuatorCommand.VALID_STEERING |
                                 ActuatorCommand.VALID_THROTTLE |
                                 ActuatorCommand.VALID_BRAKE |
                                 ActuatorCommand.VALID_TARGET_SPEED |
                                 ActuatorCommand.INTEGRITY_OK)
        self.pub.publish(output)


def main():
    rclpy.init()
    node = LongitudinalController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Intent and dynamic speed cap; existing planners retain trajectory and traffic control."""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sac_interfaces.msg import (BehaviorStatus, CameraDetection, CameraDetectionArray,
                                DetectedObject, RoadHazardArray, SpeedConstraint,
                                TrackedObjectArray, PipelineTiming, VehicleSpeed)
from std_msgs.msg import Bool, Int32, String
from tf2_ros import Buffer, TransformListener, TransformException


def follow_speed(gap, ego_speed, lead_speed, standstill_gap, headway, gain, maximum):
    if lead_speed is None:
        return 0.0
    desired = standstill_gap + headway * ego_speed
    return max(0.0, min(maximum, lead_speed + gain * (gap-desired)))


def crossing_relevant(x, y, crossing_x, longitudinal, half_width):
    return abs(x-crossing_x) <= longitudinal and abs(y) <= half_width


def pedestrian_intersects_path(x, y, lateral_speed, ego_speed, distance,
                               half_width, horizon):
    if x <= 0 or x > distance:
        return False
    if abs(y) <= half_width:
        return True
    if lateral_speed is None or y * lateral_speed >= 0:
        return False
    time_to_reach = min(horizon, x / max(ego_speed, 0.5))
    return abs(y + lateral_speed * time_to_reach) <= half_width


class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')
        defaults = dict(max_speed_mps=2.0, object_timeout=.75, camera_timeout=.75,
                        enforce_freshness=True,
                        enable_stop_sign=True,
                        speed_timeout=.25, corridor_half_width=.85,
                        crosswalk_half_width=2.0, crosswalk_longitudinal_m=3.0,
                        standstill_gap_m=2.0, time_headway_s=1.5,
                        follow_gain=0.5, stop_distance_m=12.0,
                        pedestrian_prediction_horizon=3.0,
                        future_stamp_tolerance=0.1,
                        parking_entry_id=2450,
                        parking_goal_ids=[2605, 2612, 2619, 2626, 2633, 2640, 2647, 2654],
                        parking_approach_speed_mps=1.0, parking_speed_mps=0.5)
        for key, val in defaults.items():
            self.declare_parameter(key, val)
        self.p = lambda key: self.get_parameter(key).value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tracks = None
        self.track_receipt = None
        self.camera = None
        self.camera_receipt = None
        self.speed = None
        self.speed_receipt = None
        self.odom = None
        self.odom_receipt = None
        self.switch = False
        self.goal_id = None
        self.controller_state = ''
        self.external_caps = {}
        self.hazard_receipt = None
        self.hazard_count = 0
        self.create_subscription(TrackedObjectArray, '/perception/tracked_objects',
                                 self.on_tracks, 1)
        self.create_subscription(CameraDetectionArray, '/yolo_detections',
                                 self.on_camera, 1)
        self.create_subscription(VehicleSpeed, '/vehicle/speed', self.on_speed, 1)
        self.create_subscription(Odometry, '/localization/ekf_localizer/odom',
                                 self.on_odom, 1)
        self.create_subscription(Bool, '/switch_controller', self.on_switch, 1)
        self.create_subscription(Int32, '/global_planner/goal', self.on_goal, 1)
        self.create_subscription(String, '/control/controller_state',
                                 self.on_controller_state, 1)
        for topic in ('traffic', 'road_hazard', 'map'):
            self.create_subscription(
                SpeedConstraint, f'/planning/{topic}/speed_constraint',
                lambda msg, source=topic: self.on_constraint(source, msg), 1)
        self.create_subscription(RoadHazardArray, '/perception/road_hazards',
                                 self.on_hazards, 1)
        self.speed_pub = self.create_publisher(SpeedConstraint,
                                               '/planning/dynamic/speed_constraint', 1)
        self.state_pub = self.create_publisher(BehaviorStatus,
                                               '/planning/behavior/status', 1)
        self.timing_pub = self.create_publisher(PipelineTiming,
                                                '/planning/behavior/timing', 1)
        self.timing_sequence = 0
        self.stale_count = 0
        self.create_timer(.05, self.tick)

    def valid(self, stamp, receipt, timeout):
        if stamp is None or receipt is None:
            return False
        if not self.p('enforce_freshness'):
            return True
        age = (self.get_clock().now()-Time.from_msg(stamp)).nanoseconds*1e-9
        return -self.p('future_stamp_tolerance') <= age <= timeout and time.monotonic()-receipt <= timeout

    def on_tracks(self, msg):
        self.tracks, self.track_receipt = msg, time.monotonic()

    def on_camera(self, msg):
        self.camera, self.camera_receipt = msg, time.monotonic()

    def on_speed(self, msg):
        if math.isfinite(msg.speed_mps) and msg.speed_mps >= 0:
            self.speed, self.speed_receipt = msg, time.monotonic()

    def on_odom(self, msg):
        speed = msg.twist.twist.linear.x
        if math.isfinite(speed) and speed >= 0:
            self.odom, self.odom_receipt = msg, time.monotonic()

    def on_goal(self, msg):
        if msg.data != self.goal_id:
            self.controller_state = ''
        self.goal_id = msg.data

    def on_controller_state(self, msg):
        self.controller_state = msg.data

    def on_constraint(self, source, msg):
        self.external_caps[source] = (msg, time.monotonic())

    def on_switch(self, msg):
        self.switch = msg.data

    def on_hazards(self, msg):
        self.hazard_receipt = time.monotonic()
        self.hazard_count = len(msg.hazards)

    def base_objects(self):
        if self.tracks.header.frame_id == 'base_link':
            tf = None
        else:
            try:
                tf = self.tf_buffer.lookup_transform('base_link',
                    self.tracks.header.frame_id, Time.from_msg(self.tracks.header.stamp))
            except TransformException:
                return None
        output = []
        for obj in self.tracks.objects:
            if not obj.position_valid or (self.p('enforce_freshness') and
                    (self.get_clock().now()-Time.from_msg(obj.last_seen)).nanoseconds*1e-9 > self.p('object_timeout')):
                continue
            x, y, z = obj.position.x, obj.position.y, obj.position.z
            vx = vy = None
            if tf is not None:
                q = tf.transform.rotation
                tx, ty, tz = 2*(q.y*z-q.z*y), 2*(q.z*x-q.x*z), 2*(q.x*y-q.y*x)
                x, y = (x+q.w*tx+q.y*tz-q.z*ty+tf.transform.translation.x,
                        y+q.w*ty+q.z*tx-q.x*tz+tf.transform.translation.y)
                if obj.velocity_valid:
                    vx = ((1-2*(q.y*q.y+q.z*q.z))*obj.velocity.x+
                          2*(q.x*q.y-q.z*q.w)*obj.velocity.y+
                          2*(q.x*q.z+q.y*q.w)*obj.velocity.z)
                    vy = (2*(q.x*q.y+q.z*q.w)*obj.velocity.x+
                          (1-2*(q.x*q.x+q.z*q.z))*obj.velocity.y+
                          2*(q.y*q.z-q.x*q.w)*obj.velocity.z)
            elif obj.velocity_valid:
                vx, vy = obj.velocity.x, obj.velocity.y
            output.append((obj, x, y, vx, vy))
        return output

    def tick(self):
        start = time.perf_counter()
        stamp = self.get_clock().now().to_msg()
        state, reason, limit = 'CRUISE', 'clear', self.p('max_speed_mps')
        if self.goal_id == self.p('parking_entry_id'):
            state, reason = 'PARK_APPROACH', 'route targets parking entrance'
            limit = min(limit, self.p('parking_approach_speed_mps'))
        elif self.goal_id in self.p('parking_goal_ids'):
            state, reason = 'PARKING', f'route targets parking lanelet {self.goal_id}'
            limit = min(limit, self.p('parking_speed_mps'))
            if self.controller_state.startswith('STOP: goal reached'):
                state, reason, limit = 'PARKED', 'parking goal reached', 0.0
        camera_fresh = self.camera and self.valid(
            self.camera.header.stamp, self.camera_receipt, self.p('camera_timeout'))
        speed_fresh = self.speed and self.valid(
            self.speed.header.stamp, self.speed_receipt, self.p('speed_timeout'))
        odom_fresh = self.odom and self.valid(
            self.odom.header.stamp, self.odom_receipt, self.p('speed_timeout'))
        ego_speed = (self.speed.speed_mps if speed_fresh else
                     self.odom.twist.twist.linear.x if odom_fresh else None)
        track_fresh = self.tracks and self.valid(
            self.tracks.header.stamp, self.track_receipt, self.p('object_timeout'))
        if ego_speed is None or not track_fresh:
            self.stale_count += 1
            state, reason, limit = 'EMERGENCY_STOP', 'stale vehicle speed or tracked objects', 0.0
        else:
            objects = self.base_objects()
            if objects is None:
                state, reason, limit = 'EMERGENCY_STOP', 'tracked-object transform unavailable', 0.0
                objects = []
            signs = self.camera.detections if camera_fresh else []
            crosswalk = next((d for d in signs if d.label == CameraDetection.PEDESTRIANS_CROSSING
                              and d.confidence >= .5), None)
            people, vehicles = [], []
            for item in objects:
                obj, x, y, vx, vy = item
                if x <= 0 or x > self.p('stop_distance_m'):
                    continue
                if obj.kind == DetectedObject.PERSON:
                    people.append(item)
                elif obj.kind in (DetectedObject.CAR, DetectedObject.MOTORCYCLE,
                                  DetectedObject.BUS, DetectedObject.TRUCK):
                    if abs(y) <= self.p('corridor_half_width'):
                        vehicles.append(item)
            if limit > 0 and any(pedestrian_intersects_path(
                    x, y, vy, ego_speed, self.p('stop_distance_m'),
                    self.p('corridor_half_width'),
                    self.p('pedestrian_prediction_horizon'))
                    for _, x, y, _, vy in people):
                state, reason, limit = 'PEDESTRIAN_YIELD', 'pedestrian on or moving into path', 0.0
            if limit > 0 and crosswalk is not None:
                distance = crosswalk.distance
                if math.isfinite(distance) and distance > 0:
                    relevant = [item for item in people if crossing_relevant(
                        item[1], item[2], distance,
                        self.p('crosswalk_longitudinal_m'),
                        self.p('crosswalk_half_width'))]
                    if relevant:
                        state, reason, limit = 'CROSSWALK_YIELD', 'pedestrian in crossing region', 0.0
            if vehicles and limit > 0:
                obj, gap, _, lead_v, _ = min(vehicles, key=lambda item: item[1])
                if lead_v is None:
                    state, reason, limit = 'FOLLOW_VEHICLE', 'lead velocity unavailable', 0.0
                else:
                    limit = follow_speed(
                        gap, ego_speed, lead_v,
                        self.p('standstill_gap_m'), self.p('time_headway_s'),
                        self.p('follow_gain'), limit)
                    state, reason = 'FOLLOW_VEHICLE', f'lead track {obj.track_id}, gap {gap:.1f} m'
            if self.switch and state in ('CRUISE', 'PARK_APPROACH', 'PARKING'):
                state, reason = 'OBSTACLE_AVOIDANCE', 'MissionPlanner switch active'
            if camera_fresh and state not in ('EMERGENCY_STOP', 'PEDESTRIAN_YIELD', 'CROSSWALK_YIELD'):
                if any(d.label == CameraDetection.RED_LIGHT and d.confidence >= .5 for d in signs):
                    state, reason = 'RED_LIGHT_STOP', 'traffic planner owns stop latch'
                elif self.p('enable_stop_sign') and any(
                        d.label == CameraDetection.STOP and d.confidence >= .5 for d in signs):
                    state, reason = 'STOP_SIGN', 'traffic planner owns stop lifecycle'
        cap = SpeedConstraint()
        cap.header.stamp = stamp
        cap.max_speed_mps = limit
        cap.reason = reason
        self.speed_pub.publish(cap)
        status_state, status_reason = state, reason
        stop_required = limit <= 0
        for source in ('map', 'road_hazard', 'traffic'):
            observation = self.external_caps.get(source)
            if observation is None:
                continue
            external, receipt = observation
            if not self.valid(external.header.stamp, receipt, self.p('object_timeout')):
                continue
            if external.max_speed_mps <= 0:
                stop_required = True
                if status_state not in ('EMERGENCY_STOP', 'PEDESTRIAN_YIELD',
                                        'CROSSWALK_YIELD', 'PARKED'):
                    status_state = 'TRAFFIC_STOP' if source == 'traffic' else 'EMERGENCY_STOP'
                    status_reason = f'{source}: {external.reason}'
            elif (source == 'road_hazard' and
                  external.max_speed_mps < self.p('max_speed_mps') and
                  status_state in ('CRUISE', 'PARK_APPROACH', 'PARKING')):
                status_state, status_reason = 'ROAD_HAZARD', external.reason
        if self.controller_state.startswith('STOP:'):
            stop_required = True
            if status_state in ('CRUISE', 'PARK_APPROACH', 'PARKING', 'OBSTACLE_AVOIDANCE'):
                if self.controller_state.startswith('STOP: goal reached'):
                    status_state = ('PARK_ENTRY_STOP' if self.goal_id == self.p('parking_entry_id')
                                    else 'STATION_STOP')
                else:
                    status_state = 'CONTROLLER_STOP'
                status_reason = self.controller_state
        status = BehaviorStatus()
        status.header.stamp = stamp
        status.state = status_state
        status.reason = status_reason
        status.stop_required = stop_required
        status.avoidance_active = self.switch
        self.state_pub.publish(status)
        self.timing_sequence += 1
        timing = PipelineTiming()
        timing.stamp = stamp
        timing.component = 'behavior_manager'
        timing.sequence = self.timing_sequence
        timing.execution_time_ms = (time.perf_counter()-start)*1000.0
        if track_fresh:
            timing.source_age_ms = (self.get_clock().now()-
                Time.from_msg(self.tracks.header.stamp)).nanoseconds/1e6
        timing.received_count = self.timing_sequence
        timing.processed_count = self.timing_sequence
        timing.dropped_count = self.stale_count
        self.timing_pub.publish(timing)


def main():
    rclpy.init()
    node = BehaviorManager()
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

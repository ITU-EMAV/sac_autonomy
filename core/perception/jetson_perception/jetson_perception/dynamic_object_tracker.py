"""Deterministic metric association; velocity requires an odom-frame TF."""
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from sac_interfaces.msg import DetectedObjectArray, TrackedObject, TrackedObjectArray, PipelineTiming


def transform_xy(point, tf):
    q = tf.transform.rotation
    # Rotate a 3D point using quaternion cross products.
    vx, vy, vz = point
    tx = 2*(q.y*vz-q.z*vy)
    ty = 2*(q.z*vx-q.x*vz)
    tz = 2*(q.x*vy-q.y*vx)
    return (vx+q.w*tx+q.y*tz-q.z*ty+tf.transform.translation.x,
            vy+q.w*ty+q.z*tx-q.x*tz+tf.transform.translation.y,
            vz+q.w*tz+q.x*ty-q.y*tx+tf.transform.translation.z)


class TrackStore:
    def __init__(self, gate=2.0, timeout=.8):
        self.gate, self.timeout = gate, timeout
        self.tracks = {}
        self.next_id = 1
        self.frame = None

    def update(self, observations, stamp, frame):
        if frame != self.frame:
            self.tracks.clear()
            self.frame = frame
        self.tracks = {key: val for key, val in self.tracks.items()
                       if stamp-val['last'] <= self.timeout}
        candidates = []
        for i, (kind, confidence, pos) in enumerate(observations):
            for key, track in self.tracks.items():
                dt = stamp-track['last']
                if kind != track['kind'] or dt < 0:
                    continue
                prediction = tuple(track['pos'][j]+track['vel'][j]*dt for j in range(3))
                distance = math.dist(pos, prediction)
                if distance <= self.gate:
                    candidates.append((distance, i, key))
        assigned_obs, assigned_tracks = set(), set()
        for _, i, key in sorted(candidates):
            if i in assigned_obs or key in assigned_tracks:
                continue
            assigned_obs.add(i)
            assigned_tracks.add(key)
            kind, confidence, pos = observations[i]
            track = self.tracks[key]
            dt = stamp-track['last']
            if dt > .01:
                instant = tuple((pos[j]-track['pos'][j])/dt for j in range(3))
                track['vel'] = tuple(.5*track['vel'][j]+.5*instant[j] for j in range(3))
                track['velocity_valid'] = frame == 'odom'
            track.update(pos=pos, confidence=confidence, last=stamp)
        for i, (kind, confidence, pos) in enumerate(observations):
            if i in assigned_obs:
                continue
            # Suppress duplicate same-class observations in the current frame.
            if any(kind == observations[j][0] and math.dist(pos, observations[j][2]) < .3
                   for j in assigned_obs):
                continue
            key = self.next_id
            self.next_id += 1
            self.tracks[key] = dict(kind=kind, confidence=confidence, pos=pos,
                                    vel=(0., 0., 0.), first=stamp, last=stamp,
                                    velocity_valid=False)
            assigned_obs.add(i)
        return self.tracks


class DynamicObjectTracker(Node):
    def __init__(self):
        super().__init__('dynamic_object_tracker')
        self.declare_parameter('tracking_frame', 'odom')
        self.declare_parameter('gate_m', 2.0)
        self.declare_parameter('track_timeout', .8)
        self.store = TrackStore(self.get_parameter('gate_m').value,
                                self.get_parameter('track_timeout').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(TrackedObjectArray,
                                         '/perception/tracked_objects', 1)
        self.timing_pub = self.create_publisher(PipelineTiming,
                                                '/perception/tracking/timing', 1)
        self.sequence = 0
        self.create_subscription(DetectedObjectArray, '/perception/dynamic_objects',
                                 self.on_objects, 1)

    def on_objects(self, msg):
        start = time.perf_counter()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds*1e-9
        if stamp <= 0 or self.get_clock().now().nanoseconds*1e-9-stamp > .75:
            return
        frame = self.get_parameter('tracking_frame').value
        tf = None
        try:
            tf = self.tf_buffer.lookup_transform(frame, 'base_link',
                                                 Time.from_msg(msg.header.stamp))
        except TransformException:
            frame = 'base_link'
        observations = []
        for item in msg.objects:
            if not item.position_valid or not item.base_position_valid:
                continue
            pos = (item.base_position.x, item.base_position.y, item.base_position.z)
            if tf is not None:
                pos = transform_xy(pos, tf)
            if all(math.isfinite(v) for v in pos):
                observations.append((item.kind, item.confidence, pos))
        tracks = self.store.update(observations, stamp, frame)
        output = TrackedObjectArray()
        output.header = msg.header
        output.header.frame_id = frame
        for key, state in tracks.items():
            item = TrackedObject()
            item.track_id = key
            item.kind = state['kind']
            item.confidence = state['confidence']
            item.position_valid = True
            item.position.x, item.position.y, item.position.z = state['pos']
            item.velocity_valid = state['velocity_valid'] and frame == 'odom'
            item.velocity.x, item.velocity.y, item.velocity.z = state['vel']
            item.age_sec = max(0., stamp-state['first'])
            item.last_seen = Time(seconds=state['last']).to_msg()
            item.frame_id = frame
            output.objects.append(item)
        self.pub.publish(output)
        self.sequence += 1
        timing = PipelineTiming()
        timing.stamp = self.get_clock().now().to_msg()
        timing.component = 'dynamic_object_tracker'
        timing.sequence = self.sequence
        timing.execution_time_ms = (time.perf_counter()-start)*1000
        timing.source_age_ms = (self.get_clock().now().nanoseconds*1e-9-stamp)*1000
        timing.processed_count = self.sequence
        self.timing_pub.publish(timing)


def main():
    rclpy.init()
    node = DynamicObjectTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

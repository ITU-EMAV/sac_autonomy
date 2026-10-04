#!/usr/bin/env python3
"""Relate NDT execution spikes to iteration counts in a ROS 2 bag."""

import sqlite3
import sys
from bisect import bisect_left

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def summarize(items):
    values = sorted(items)
    if not values:
        return "none"
    return (
        f"n={len(values)} median={values[len(values) // 2]:.1f} "
        f"p95={values[round((len(values) - 1) * .95)]:.1f} "
        f"max={values[-1]:.1f}"
    )


conn = sqlite3.connect(f"file:{sys.argv[1]}?mode=ro", uri=True)
topics = {name: (topic_id, get_message(message_type)) for topic_id, name, message_type
          in conn.execute("SELECT id, name, type FROM topics")}
names = {
    "exe": "/localization/ndt_localizer/debug/exe_time_ms",
    "iterations": "/localization/ndt_localizer/debug/iteration_num",
}
data = {}
for key, name in names.items():
    topic_id, message_type = topics[name]
    data[key] = [
        (recorded_ns, deserialize_message(blob, message_type).data)
        for recorded_ns, blob in conn.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,))
    ]
pose_id, pose_type = topics["/localization/ekf_localizer/pose_with_cov"]
poses = [
    (recorded_ns, msg.pose.pose.position.x, msg.pose.pose.position.y)
    for recorded_ns, blob in conn.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (pose_id,))
    for msg in [deserialize_message(blob, pose_type)]
]
pose_times = [row[0] for row in poses]
ndt_pose_id, ndt_pose_type = topics["/localization/ndt_localizer/pose_with_cov"]
ndt_pose_times = [row[0] for row in conn.execute(
    "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
    (ndt_pose_id,))]
exe = data["exe"]
iters = data["iterations"]
print("execution_ms", summarize([value for _, value in exe]))
print("over_100ms", sum(value > 100 for _, value in exe))
print("over_200ms", sum(value > 200 for _, value in exe))
print("over_500ms", sum(value > 500 for _, value in exe))
by_iteration = {}
for (time_ns, ms), (iter_time_ns, count) in zip(exe, iters):
    if abs(time_ns - iter_time_ns) < 50_000_000:
        by_iteration.setdefault(count, []).append(ms)
for count, values in sorted(by_iteration.items()):
    if len(values) >= 10:
        print(f"iterations={count}", summarize(values))
for threshold in (20, 40):
    matches = 0
    total = 0
    for (time_ns, _), (_, count) in zip(exe, iters):
        if count < threshold:
            continue
        total += 1
        index = bisect_left(ndt_pose_times, time_ns)
        nearby = ndt_pose_times[max(0, index - 1):index + 1]
        matches += bool(nearby and min(abs(t - time_ns) for t in nearby) < 50_000_000)
    print(f"iterations_ge_{threshold}: output_near_debug={matches}/{total}")
print("worst_spikes:")
for time_ns, value in sorted(exe, key=lambda row: row[1], reverse=True)[:12]:
    index = min(bisect_left(pose_times, time_ns), len(poses) - 1)
    _, x, y = poses[index]
    print(f"  t={(time_ns - exe[0][0]) / 1e9:.1f}s ms={value:.1f} xy=({x:.1f},{y:.1f})")

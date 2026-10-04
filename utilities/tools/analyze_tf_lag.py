#!/usr/bin/env python3
"""Summarize message age and TF freshness from a ROS 2 SQLite bag."""

import sqlite3
import sys
from collections import defaultdict

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def quantile(values, fraction):
    values = sorted(values)
    return values[round((len(values) - 1) * fraction)]


def summarize(values):
    if not values:
        return "none"
    return (
        f"n={len(values)} min={min(values):.1f} "
        f"p50={quantile(values, 0.5):.1f} "
        f"p95={quantile(values, 0.95):.1f} max={max(values):.1f}"
    )


def main(path):
    connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    topics = {
        row[0]: (row[1], get_message(row[2]))
        for row in connection.execute("SELECT id, name, type FROM topics")
    }
    ages = defaultdict(list)
    times = defaultdict(list)
    values = defaultdict(list)
    xy = defaultdict(list)
    moving_times = []
    for topic_id, recorded_ns, blob in connection.execute(
        "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
    ):
        name, message_type = topics[topic_id]
        if name not in {
            "/tf", "/velodyne_points", "/sensing/points_filtered",
            "/encoder_speed", "/zed/zed_node/odom",
            "/zed/zed_node/imu/data", "/localization/ndt_localizer/pose_with_cov",
            "/localization/ekf_localizer/pose_with_cov",
            "/localization/ekf_localizer/twist_with_cov",
            "/localization/ndt_localizer/debug/exe_time_ms",
            "/localization/ndt_localizer/debug/iteration_num",
        }:
            continue
        message = deserialize_message(blob, message_type)
        if name == "/tf":
            for transform in message.transforms:
                key = f"TF {transform.header.frame_id}->{transform.child_frame_id}"
                ages[key].append((recorded_ns - stamp_ns(transform.header.stamp)) / 1e6)
                times[key].append(recorded_ns)
                xy[key].append((transform.transform.translation.x, transform.transform.translation.y))
        else:
            if hasattr(message, "header"):
                ages[name].append((recorded_ns - stamp_ns(message.header.stamp)) / 1e6)
            times[name].append(recorded_ns)
            if (name == "/encoder_speed" or
                    name.startswith("/localization/ndt_localizer/debug/")) and hasattr(message, "data"):
                values[name].append(float(message.data))
                if name == "/encoder_speed" and abs(message.data) > 0.2:
                    moving_times.append(recorded_ns)
            if hasattr(message, "pose"):
                pose = message.pose.pose if hasattr(message.pose, "pose") else message.pose
                xy[name].append((pose.position.x, pose.position.y))

    for name, event_times in times.items():
        intervals = [
            (b - a) / 1e6 for a, b in zip(event_times, event_times[1:])
        ]
        print(name)
        print(f"  age_ms: {summarize(ages[name])}")
        print(f"  interval_ms: {summarize(intervals)}")
        if values[name]:
            print(f"  data: {summarize(values[name])}")
        if xy[name]:
            first, last = xy[name][0], xy[name][-1]
            print(f"  xy_first_last: {first} -> {last}")
        if name == "TF map->base_link":
            late = [
                (event_time - event_times[0]) / 1e9
                for event_time, age in zip(event_times, ages[name]) if age > 500.0
            ]
            print(f"  age_gt_500ms: {len(late)}")
            if late:
                print(f"  first_last_late_s: {late[0]:.1f} -> {late[-1]:.1f}")
    if moving_times:
        print("moving_speed_gt_0.2_mps")
        print(f"  samples={len(moving_times)} duration_s={(moving_times[-1] - moving_times[0]) / 1e9:.1f}")


if __name__ == "__main__":
    main(sys.argv[1])

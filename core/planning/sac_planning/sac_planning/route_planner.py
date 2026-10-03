"""Route planner: follows a route given on the world map.

The first planning method: the waypoints of a route file (latitude/longitude, see route.py)
are converted to the map frame with the site's datum, smoothed and resampled, and published
as the path to follow. Other planning methods publish the same topic.

Publishes (latched, i.e. transient local):
  ~/path     nav_msgs/Path in `map`, a pose every `spacing` metres (remapped to
             /sac/planning/path). A loop's path does not repeat its first pose.
  ~/geojson  foxglove_msgs/GeoJSON: the route and its waypoints, for a Map panel

Parameters:
  route              route file: a path, or a file name in this package's routes/
  datum.latitude, datum.longitude, datum.altitude, datum.heading
                     origin of the map frame (see geodesy.py)
  spacing            distance between path poses [m]
  smoothing          corner-rounding passes over the waypoints (0: straight lines)
  reverse            drive the route backwards
"""

import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from foxglove_msgs.msg import GeoJSON
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from sac_planning.geodesy import MapFrame
from sac_planning.route import headings, load_route, resample, smooth


class RoutePlanner(Node):
    def __init__(self):
        super().__init__("route_planner")
        route_file = self.declare_parameter("route", "").value
        datum = MapFrame(
            self.declare_parameter("datum.latitude", 0.0).value,
            self.declare_parameter("datum.longitude", 0.0).value,
            self.declare_parameter("datum.altitude", 0.0).value,
            self.declare_parameter("datum.heading", 0.0).value,
        )
        spacing = self.declare_parameter("spacing", 0.5).value
        smoothing = self.declare_parameter("smoothing", 2).value
        reverse = self.declare_parameter("reverse", False).value
        self.frame_id = self.declare_parameter("frame_id", "map").value

        path = self.find_route(route_file)
        route = load_route(path)
        waypoints = route.waypoints[::-1] if reverse else route.waypoints
        local = [datum.to_map(lat, lon)[:2] for lat, lon in waypoints]
        points = resample(smooth(local, route.closed, smoothing), route.closed, spacing)
        yaws = headings(points, route.closed)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_publisher = self.create_publisher(Path, "~/path", latched)
        self.geojson_publisher = self.create_publisher(GeoJSON, "~/geojson", latched)
        self.path_publisher.publish(self.make_path(points, yaws))
        self.geojson_publisher.publish(self.make_geojson(route, waypoints, points, datum))

        length = len(points) * spacing
        self.get_logger().info(
            f"Route '{route.name}' ({path}): {len(waypoints)} waypoints, "
            f"{'loop' if route.closed else 'open'}, {length:.0f} m"
        )

    def find_route(self, name):
        if not name:
            raise ValueError("Set the route parameter to a route file")
        if os.path.isabs(name) or os.path.exists(name):
            return name
        return os.path.join(get_package_share_directory("sac_planning"), "routes", name)

    def make_path(self, points, yaws):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        for (x, y), yaw in zip(points, yaws):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(pose)
        return path

    def make_geojson(self, route, waypoints, points, datum):
        line = [list(datum.to_geodetic(x, y)[1::-1]) for x, y in points[::4]]
        if route.closed:
            line.append(line[0])
        features = [
            {
                "type": "Feature",
                "properties": {"name": route.name, "style": {"color": "#1e90ff", "weight": 3}},
                "geometry": {"type": "LineString", "coordinates": line},
            }
        ] + [
            {
                "type": "Feature",
                "properties": {"name": f"waypoint {i}", "style": {"color": "#ff8c00", "radius": 3}},
                "geometry": {"type": "Point", "coordinates": [lon, lat]},
            }
            for i, (lat, lon) in enumerate(waypoints)
        ]
        return GeoJSON(geojson=json.dumps({"type": "FeatureCollection", "features": features}))


def main():
    rclpy.init()
    node = RoutePlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

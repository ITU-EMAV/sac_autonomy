"""Routes: waypoints given on the world map (latitude/longitude), turned into a smooth path.

A route file is one of:
  .geojson  a LineString (e.g. drawn on https://geojson.io), or Point features in the order
            they should be driven. GeoJSON coordinates are [longitude, latitude].
  .csv      one "latitude,longitude" per line; a header line and "#" comments are skipped.

A route whose last waypoint is within `closing_distance` of its first is a loop (a lap).
"""

import json
import math
import os

import numpy as np


class Route:
    def __init__(self, waypoints, name="", closed=False):
        self.waypoints = waypoints  # [(latitude, longitude), ...]
        self.name = name
        self.closed = closed


def load_route(path, closing_distance=5.0):
    if path.endswith(".geojson") or path.endswith(".json"):
        waypoints, name = _read_geojson(path)
    elif path.endswith(".csv"):
        waypoints, name = _read_csv(path), ""
    else:
        raise ValueError(f"Unknown route format (use .geojson or .csv): {path}")
    if len(waypoints) < 2:
        raise ValueError(f"A route needs at least 2 waypoints: {path}")
    name = name or os.path.splitext(os.path.basename(path))[0]

    closed = _distance(waypoints[0], waypoints[-1]) < closing_distance
    if closed and len(waypoints) > 2:
        waypoints = waypoints[:-1]  # the loop closes by itself
    return Route(waypoints, name, closed)


def _read_geojson(path):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    features = data["features"] if data.get("type") == "FeatureCollection" else [data]
    lines, points, name = [], [], data.get("name", "")
    for feature in features:
        geometry = feature.get("geometry", feature)
        if geometry["type"] == "LineString":
            lines.append([(c[1], c[0]) for c in geometry["coordinates"]])
            name = name or feature.get("properties", {}).get("name", "")
        elif geometry["type"] == "Point":
            c = geometry["coordinates"]
            points.append((c[1], c[0]))
    if len(lines) > 1:
        raise ValueError(f"{path}: more than one LineString; a route is one line")
    return (lines[0] if lines else points), name


def _read_csv(path):
    waypoints = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.split("#")[0].strip()
            if not line:
                continue
            fields = line.replace(";", ",").split(",")
            try:
                waypoints.append((float(fields[0]), float(fields[1])))
            except ValueError:
                if waypoints:
                    raise  # only the first line may be a header
    return waypoints


def _distance(a, b):
    """Approximate distance between two (latitude, longitude) points [m]."""
    dy = math.radians(b[0] - a[0]) * 6371000.0
    dx = math.radians(b[1] - a[1]) * 6371000.0 * math.cos(math.radians(a[0]))
    return math.hypot(dx, dy)


def smooth(points, closed, iterations):
    """Chaikin corner cutting: rounds the corners between waypoints (the path stays inside
    them); the end points of an open route stay where they are."""
    p = np.asarray(points, dtype=float)
    for _ in range(iterations):
        if closed:
            a, b = p, np.roll(p, -1, axis=0)
        else:
            a, b = p[:-1], p[1:]
        q = np.empty((2 * len(a), p.shape[1]))
        q[0::2] = 0.75 * a + 0.25 * b
        q[1::2] = 0.25 * a + 0.75 * b
        p = q if closed else np.vstack([p[:1], q, p[-1:]])
    return p


def resample(points, closed, spacing):
    """Points every `spacing` metres along the polyline (a loop ends one step before its
    start)."""
    p = np.asarray(points, dtype=float)
    if closed:
        p = np.vstack([p, p[:1]])
    lengths = np.hypot(*np.diff(p[:, :2], axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(lengths)])
    count = max(2, int(round(s[-1] / spacing)))
    targets = np.linspace(0.0, s[-1], count, endpoint=not closed)
    return np.column_stack([np.interp(targets, s, p[:, i]) for i in range(p.shape[1])])


def headings(points, closed):
    """Direction of travel at each point [rad]."""
    p = np.asarray(points, dtype=float)
    after = np.roll(p, -1, axis=0) if closed else np.vstack([p[1:], 2 * p[-1:] - p[-2:-1]])
    before = np.roll(p, 1, axis=0) if closed else np.vstack([2 * p[:1] - p[1:2], p[:-1]])
    d = after - before
    return np.arctan2(d[:, 1], d[:, 0])

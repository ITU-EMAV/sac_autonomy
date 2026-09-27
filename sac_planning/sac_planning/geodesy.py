"""Conversion between latitude/longitude and the map frame.

The map frame is a local tangent plane (ENU: x east, y north, z up) at a datum, turned
counter-clockwise by the datum's heading. This is Gazebo's convention for a world with
<spherical_coordinates> (world_frame_orientation ENU, heading_deg), so in the simulation the
datum is the world's spherical coordinates. On the real car the localization uses the same
datum, so routes given as latitude/longitude land in the same place in both.

Exact on the WGS84 ellipsoid (through ECEF), no flat-earth approximation.
"""

import math

import numpy as np

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def geodetic_to_ecef(latitude, longitude, altitude=0.0):
    lat, lon = math.radians(latitude), math.radians(longitude)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    return np.array(
        [
            (n + altitude) * math.cos(lat) * math.cos(lon),
            (n + altitude) * math.cos(lat) * math.sin(lon),
            (n * (1 - WGS84_E2) + altitude) * math.sin(lat),
        ]
    )


def ecef_to_geodetic(x, y, z):
    """Bowring's method; sub-millimetre near the Earth's surface."""
    b = WGS84_A * (1 - WGS84_F)
    ep2 = (WGS84_A**2 - b**2) / b**2
    p = math.hypot(x, y)
    theta = math.atan2(z * WGS84_A, p * b)
    lat = math.atan2(z + ep2 * b * math.sin(theta) ** 3, p - WGS84_E2 * WGS84_A * math.cos(theta) ** 3)
    lon = math.atan2(y, x)
    n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
    altitude = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), altitude


class MapFrame:
    """The map frame of a datum.

    latitude, longitude [deg], altitude [m]: where the map origin is.
    heading [deg]: counter-clockwise angle from east to the map's x axis (Gazebo's heading_deg).
    """

    def __init__(self, latitude, longitude, altitude=0.0, heading=0.0):
        self.origin = geodetic_to_ecef(latitude, longitude, altitude)
        lat, lon = math.radians(latitude), math.radians(longitude)
        # Rows: east, north, up in ECEF
        enu_from_ecef = np.array(
            [
                [-math.sin(lon), math.cos(lon), 0.0],
                [-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat)],
                [math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)],
            ]
        )
        h = math.radians(heading)
        map_from_enu = np.array(
            [[math.cos(h), math.sin(h), 0.0], [-math.sin(h), math.cos(h), 0.0], [0.0, 0.0, 1.0]]
        )
        self.map_from_ecef = map_from_enu @ enu_from_ecef

    def to_map(self, latitude, longitude, altitude=0.0):
        """(x, y, z) in the map frame [m]."""
        return self.map_from_ecef @ (geodetic_to_ecef(latitude, longitude, altitude) - self.origin)

    def to_geodetic(self, x, y, z=0.0):
        """(latitude, longitude, altitude) of a map point."""
        return ecef_to_geodetic(*(self.map_from_ecef.T @ np.array([x, y, z]) + self.origin))

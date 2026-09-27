// Latitude/longitude <-> map frame, the same convention as sac_planning's geodesy.py:
// a local tangent plane (ENU) at a datum, turned counter-clockwise by the datum's heading
// (Gazebo's <spherical_coordinates> heading_deg). Exact on the WGS84 ellipsoid.

#pragma once

#include <Eigen/Dense>

namespace sac_localization
{

struct Datum
{
  double latitude = 0.0;   // [deg]
  double longitude = 0.0;  // [deg]
  double altitude = 0.0;   // [m]
  double heading = 0.0;    // [deg] counter-clockwise from east to the map's x axis
};

class MapFrame
{
public:
  explicit MapFrame(const Datum & datum);

  /// (x, y, z) in the map frame [m].
  Eigen::Vector3d toMap(double latitude, double longitude, double altitude) const;
  /// (latitude, longitude, altitude) of a map point.
  Eigen::Vector3d toGeodetic(const Eigen::Vector3d & map_point) const;
  /// Rotation from ENU to the map frame (for velocities and headings).
  const Eigen::Matrix3d & mapFromEnu() const { return map_from_enu_; }

private:
  Eigen::Vector3d origin_ecef_;
  Eigen::Matrix3d map_from_ecef_;
  Eigen::Matrix3d map_from_enu_;
};

}  // namespace sac_localization

#include "sac_localization/core/geodesy.hpp"

#include <cmath>

namespace sac_localization
{

namespace
{
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);
constexpr double kDegree = M_PI / 180.0;

Eigen::Vector3d geodeticToEcef(double latitude, double longitude, double altitude)
{
  const double lat = latitude * kDegree;
  const double lon = longitude * kDegree;
  const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
  return {
    (n + altitude) * std::cos(lat) * std::cos(lon),
    (n + altitude) * std::cos(lat) * std::sin(lon),
    (n * (1.0 - kE2) + altitude) * std::sin(lat)};
}

/// Bowring's method; sub-millimetre near the Earth's surface.
Eigen::Vector3d ecefToGeodetic(const Eigen::Vector3d & ecef)
{
  const double b = kA * (1.0 - kF);
  const double ep2 = (kA * kA - b * b) / (b * b);
  const double p = std::hypot(ecef.x(), ecef.y());
  const double theta = std::atan2(ecef.z() * kA, p * b);
  const double lat = std::atan2(
    ecef.z() + ep2 * b * std::pow(std::sin(theta), 3), p - kE2 * kA * std::pow(std::cos(theta), 3));
  const double lon = std::atan2(ecef.y(), ecef.x());
  const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
  return {lat / kDegree, lon / kDegree, p / std::cos(lat) - n};
}
}  // namespace

MapFrame::MapFrame(const Datum & datum)
{
  origin_ecef_ = geodeticToEcef(datum.latitude, datum.longitude, datum.altitude);
  const double lat = datum.latitude * kDegree;
  const double lon = datum.longitude * kDegree;
  Eigen::Matrix3d enu_from_ecef;
  enu_from_ecef << -std::sin(lon), std::cos(lon), 0.0,
    -std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat),
    std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat);
  const double h = datum.heading * kDegree;
  // The map's x axis is `heading` counter-clockwise from east
  map_from_enu_ << std::cos(h), std::sin(h), 0.0, -std::sin(h), std::cos(h), 0.0, 0.0, 0.0, 1.0;
  map_from_ecef_ = map_from_enu_ * enu_from_ecef;
}

Eigen::Vector3d MapFrame::toMap(double latitude, double longitude, double altitude) const
{
  return map_from_ecef_ * (geodeticToEcef(latitude, longitude, altitude) - origin_ecef_);
}

Eigen::Vector3d MapFrame::toGeodetic(const Eigen::Vector3d & map_point) const
{
  return ecefToGeodetic(map_from_ecef_.transpose() * map_point + origin_ecef_);
}

}  // namespace sac_localization

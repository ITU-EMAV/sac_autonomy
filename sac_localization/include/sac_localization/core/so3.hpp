// 3D rotation helpers: exponential and logarithm maps between rotation vectors and
// quaternions, and the skew-symmetric matrix.

#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace sac_localization
{

inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

/// Rotation vector (axis * angle) -> unit quaternion.
inline Eigen::Quaterniond expSO3(const Eigen::Vector3d & rotation_vector)
{
  const double angle = rotation_vector.norm();
  if (angle < 1e-10) {
    Eigen::Quaterniond q(1.0, 0.5 * rotation_vector.x(), 0.5 * rotation_vector.y(), 0.5 * rotation_vector.z());
    return q.normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

/// Unit quaternion -> rotation vector with an angle in [0, pi].
inline Eigen::Vector3d logSO3(const Eigen::Quaterniond & rotation)
{
  Eigen::Quaterniond q = rotation.normalized();
  if (q.w() < 0.0) {
    q.coeffs() = -q.coeffs();
  }
  const Eigen::Vector3d v = q.vec();
  const double sin_half = v.norm();
  if (sin_half < 1e-10) {
    return 2.0 * v;
  }
  const double angle = 2.0 * std::atan2(sin_half, q.w());
  return v * (angle / sin_half);
}

/// Yaw (rotation about z) of a rotation, ZYX convention.
inline double yawOf(const Eigen::Quaterniond & q)
{
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// Angle wrapped to [-pi, pi].
inline double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace sac_localization

// Shared pieces of the motion models: rigid-body integration and block-wise random-walk
// process noise.

#pragma once

#include <string>

#include <Eigen/Dense>

#include "sac_localization/core/params.hpp"
#include "sac_localization/core/state.hpp"

namespace sac_localization
{

/// Random-walk noise of each block, as standard deviations per sqrt(second) (the variance
/// grows by value^2 * dt). Parameters (motion_model.*), 3 values per axis:
///   position_noise, orientation_noise, velocity_noise, angular_velocity_noise,
///   acceleration_noise; and one value each: gyro_bias_noise, accel_bias_noise,
///   other_block_noise (blocks of other plugins).
struct BlockNoise
{
  Eigen::Vector3d position{0.05, 0.05, 0.05};
  Eigen::Vector3d orientation{0.01, 0.01, 0.01};
  Eigen::Vector3d velocity{0.2, 0.2, 0.2};
  Eigen::Vector3d angular_velocity{0.3, 0.3, 0.6};
  Eigen::Vector3d acceleration{3.0, 3.0, 1.0};
  double gyro_bias = 1e-4;
  double accel_bias = 1e-3;
  double other = 1e-4;

  void read(const Params & params);
  /// Diagonal Q over dt for every block of the layout.
  Eigen::MatrixXd covariance(const StateLayout & layout, double dt) const;
};

/// Moves a rigid body for dt with body-frame velocity v, angular velocity w and acceleration
/// a (without gravity), writing position, orientation and velocity into `state`.
void integrateBody(
  State & state, const Eigen::Vector3d & v, const Eigen::Vector3d & w, const Eigen::Vector3d & a, double dt);

}  // namespace sac_localization

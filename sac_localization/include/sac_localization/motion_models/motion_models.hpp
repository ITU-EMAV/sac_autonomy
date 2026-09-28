// The built-in motion models (plugins "constant_acceleration", "imu_driven",
// "kinematic_bicycle").

#pragma once

#include <string>
#include <vector>

#include "sac_localization/core/motion_model.hpp"
#include "sac_localization/motion_models/common.hpp"

namespace sac_localization
{

/// 3D constant acceleration and angular velocity, no inputs: everything comes from the
/// measurements. Works with any sensor set.
///   v' = v + (a - w x v) dt,  p' = p + R (v dt + (a - w x v) dt^2 / 2),  q' = q Exp(w dt)
class ConstantAcceleration : public MotionModel
{
public:
  void initialize(const Params & params) override;
  State predict(const State & x, double dt, const Inputs & u) const override;
  Eigen::MatrixXd processNoise(const State & x, double dt, const Inputs & u) const override;

protected:
  BlockNoise noise_;
};

/// Strapdown inertial model: an IMU (adapter with as_input: true, readings rotated to the
/// body frame) drives the prediction. w = gyro - gyro_bias, a = f - accel_bias - R^T g.
/// Parameters: input (the IMU's sensor name), gravity, gyro_noise and accel_noise (used when
/// the input has no covariance), plus BlockNoise. Without an input yet it predicts like
/// ConstantAcceleration.
class ImuDriven : public ConstantAcceleration
{
public:
  void initialize(const Params & params) override;
  void addStates(StateLayoutBuilder & builder) const override;
  std::vector<std::string> inputs() const override { return {input_}; }
  State predict(const State & x, double dt, const Inputs & u) const override;
  Eigen::MatrixXd processNoise(const State & x, double dt, const Inputs & u) const override;

private:
  std::string input_;
  double gravity_ = 9.80665;
  double gyro_noise_ = 0.002;
  double accel_noise_ = 0.02;
};

/// Kinematic bicycle (car) model: the rear-axle speed and the steering angle (wheel adapter
/// with as_input: true) give v and the yaw rate (v tan(steering) / wheel_base); roll, pitch
/// and height follow a random walk that the IMUs and GNSS correct. Parameters: input,
/// wheel_base, speed_noise, steering_noise, plus BlockNoise. Without an input yet it predicts
/// like ConstantAcceleration.
class KinematicBicycle : public ConstantAcceleration
{
public:
  void initialize(const Params & params) override;
  std::vector<std::string> inputs() const override { return {input_}; }
  State predict(const State & x, double dt, const Inputs & u) const override;
  Eigen::MatrixXd processNoise(const State & x, double dt, const Inputs & u) const override;

private:
  std::string input_;
  double wheel_base_ = 1.873;
  double speed_noise_ = 0.1;
  double steering_noise_ = 0.01;
};

}  // namespace sac_localization

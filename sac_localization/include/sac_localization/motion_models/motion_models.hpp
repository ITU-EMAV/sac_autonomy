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

/// Dynamic bicycle (single-track) model: like KinematicBicycle, the wheels' speed and
/// steering (as_input) drive it, but the tyres slip. Lateral forces are linear in the slip
/// angles (cornering stiffness), so the car's lateral velocity and yaw rate follow the
/// steering with a lag and it understeers or oversteers as a real car does.
///
///   lateral velocity v and yaw rate r at the centre of mass, speed u:
///     m (v' + u r) = Fyf + Fyr,  Iz r' = lf Fyf - lr Fyr
///     Fyf = Cf (steering - (v + lf r) / u),  Fyr = Cr (-(v - lr r) / u)
///
/// Integrated with backward Euler (stable at low speed, where the 1/u terms make forward
/// Euler blow up; Ge et al., "Numerically Stable Dynamic Bicycle Model for Discrete-time
/// Control", 2021) and blended into the kinematic model below `dynamic_speed` (fully
/// kinematic under `kinematic_speed`), as racing stacks do.
/// Parameters: input, mass, yaw_inertia, lf, lr (centre of mass to the front / rear axle),
/// cornering_stiffness_front / _rear [N/rad per axle], kinematic_speed, dynamic_speed,
/// speed_noise, steering_noise, lateral_noise, plus BlockNoise.
///   speed_from: state (default) the speed is the state's, with constant longitudinal
///               acceleration, and the wheel speed a measurement (wheel adapter
///               as_input + also_measure, use: [speed]); input: the wheel speed input as it
///               is. With `input`, a spinning or airborne wheel (crest landings, wheelspin,
///               locked brakes) drives the estimate off, as it cannot be rejected.
///
/// The cornering stiffness, cornering_stiffness_mode:
///   manual (default): cornering_stiffness_front / _rear as they are
///   estimate: learnt while driving, starting from those values. The state gets a block
///     "cornering_stiffness" with the logarithm of the factor on them (they stay positive);
///     the gyros, accelerometers and GNSS correct it through the yaw rate and sideways slip
///     the model predicts in corners. The estimate is in the status output. It needs a
///     precise heading (RTK dual-antenna GNSS): with 1 m GNSS it settles too soft (DESIGN.md).
///       cornering_stiffness_estimate: grip (default; one factor on both axles, the road's
///         grip: dry to wet) or front_rear (one per axle; the balance needs varied corners)
///       cornering_stiffness_uncertainty: initial standard deviation of the log factor
///         (0.5: about x0.6 to x1.6)
///       cornering_stiffness_noise: its random walk per sqrt(s) (0.005)
///       max_cornering_stiffness_factor: the factor stays within 1/value..value (5)
class DynamicBicycle : public ConstantAcceleration
{
public:
  static constexpr char kStiffnessBlock[] = "cornering_stiffness";

  void initialize(const Params & params) override;
  void addStates(StateLayoutBuilder & builder) const override;
  void initializeBelief(Belief & belief) const override;
  std::vector<EstimatedParameter> estimatedParameters(const Belief & belief) const override;
  std::vector<std::string> inputs() const override { return {input_}; }
  State predict(const State & x, double dt, const Inputs & u) const override;
  Eigen::MatrixXd processNoise(const State & x, double dt, const Inputs & u) const override;

private:
  enum class Estimate { kNone, kGrip, kFrontRear };
  /// The cornering stiffness [front, rear] in state x
  Eigen::Vector2d corneringStiffness(const State & x) const;

  Estimate estimate_ = Estimate::kNone;
  bool initialized_ = false;  // the mode is read once: it shapes the state
  double stiffness_uncertainty_ = 0.5;
  double stiffness_noise_ = 0.005;
  double max_stiffness_factor_ = 5.0;
  std::string input_;
  double mass_ = 910.0;
  double yaw_inertia_ = 800.0;
  double lf_ = 1.124;
  double lr_ = 0.749;
  double cf_ = 36000.0;
  double cr_ = 52000.0;
  double kinematic_speed_ = 0.5;
  double dynamic_speed_ = 2.0;
  double speed_noise_ = 0.1;
  double steering_noise_ = 0.01;
  double lateral_noise_ = 0.3;
  bool speed_from_state_ = true;
};

}  // namespace sac_localization

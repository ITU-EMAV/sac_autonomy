#include "sac_localization/motion_models/motion_models.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "sac_localization/core/so3.hpp"

namespace sac_localization
{

namespace
{
Eigen::Vector3d readVector(const Params & params, const std::string & key, const Eigen::Vector3d & fallback)
{
  const std::vector<double> v = params.getDoubles(key, {fallback.x(), fallback.y(), fallback.z()});
  if (v.size() == 1) {
    return Eigen::Vector3d::Constant(v[0]);
  }
  if (v.size() != 3) {
    throw std::invalid_argument("motion_model." + key + " needs 1 or 3 values");
  }
  return {v[0], v[1], v[2]};
}

bool endsWith(const std::string & text, const std::string & suffix)
{
  return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void addVariance(Eigen::MatrixXd & Q, const StateLayout & layout, const std::string & block, const Eigen::Vector3d & variance)
{
  const int o = layout.block(block).tangent_offset;
  for (int i = 0; i < 3; ++i) {
    Q(o + i, o + i) += variance(i);
  }
}
}  // namespace

// ---------------------------------------------------------------- common
void BlockNoise::read(const Params & params)
{
  position = readVector(params, "position_noise", position);
  orientation = readVector(params, "orientation_noise", orientation);
  velocity = readVector(params, "velocity_noise", velocity);
  angular_velocity = readVector(params, "angular_velocity_noise", angular_velocity);
  acceleration = readVector(params, "acceleration_noise", acceleration);
  gyro_bias = params.getDouble("gyro_bias_noise", gyro_bias);
  accel_bias = params.getDouble("accel_bias_noise", accel_bias);
  other = params.getDouble("other_block_noise", other);
}

Eigen::MatrixXd BlockNoise::covariance(const StateLayout & layout, double dt) const
{
  Eigen::VectorXd diagonal(layout.tangentSize());
  for (const BlockInfo & b : layout.blocks()) {
    Eigen::VectorXd sigma = Eigen::VectorXd::Constant(b.tangent_size, other);
    if (b.name == blocks::kPosition) {
      sigma = position;
    } else if (b.name == blocks::kOrientation) {
      sigma = orientation;
    } else if (b.name == blocks::kLinearVelocity) {
      sigma = velocity;
    } else if (b.name == blocks::kAngularVelocity) {
      sigma = angular_velocity;
    } else if (b.name == blocks::kLinearAcceleration) {
      sigma = acceleration;
    } else if (endsWith(b.name, "gyro_bias")) {
      sigma.setConstant(gyro_bias);
    } else if (endsWith(b.name, "accel_bias")) {
      sigma.setConstant(accel_bias);
    }
    diagonal.segment(b.tangent_offset, b.tangent_size) = sigma.array().square() * dt;
  }
  return diagonal.asDiagonal();
}

void integrateBody(
  State & state, const Eigen::Vector3d & v, const Eigen::Vector3d & w, const Eigen::Vector3d & a, double dt)
{
  const Eigen::Quaterniond q = state.orientation();
  const Eigen::Vector3d v_dot = a - w.cross(v);  // body-frame velocity changes as the body turns
  state.vector(blocks::kPosition) = state.position() + q * (v * dt + 0.5 * v_dot * dt * dt);
  state.vector(blocks::kLinearVelocity) = v + v_dot * dt;
  state.rotation(blocks::kOrientation) = (q * expSO3(w * dt)).normalized();
}

// ---------------------------------------------------------------- constant acceleration
void ConstantAcceleration::initialize(const Params & params)
{
  noise_.read(params);
}

State ConstantAcceleration::predict(const State & x, double dt, const Inputs &) const
{
  State y = x;
  integrateBody(y, x.linearVelocity(), x.angularVelocity(), x.linearAcceleration(), dt);
  return y;
}

Eigen::MatrixXd ConstantAcceleration::processNoise(const State & x, double dt, const Inputs &) const
{
  return noise_.covariance(x.layout(), dt);
}

// ---------------------------------------------------------------- IMU driven
void ImuDriven::initialize(const Params & params)
{
  // Smaller defaults than ConstantAcceleration: the IMU carries the motion
  noise_.position = {0.02, 0.02, 0.02};
  noise_.orientation = {0.001, 0.001, 0.001};
  noise_.velocity = {0.05, 0.05, 0.05};
  noise_.read(params);
  input_ = params.getString("input", "");
  if (input_.empty()) {
    throw std::invalid_argument("motion_model.input: the IMU sensor that drives imu_driven");
  }
  gravity_ = params.getDouble("gravity", gravity_);
  gyro_noise_ = params.getDouble("gyro_noise", gyro_noise_);
  accel_noise_ = params.getDouble("accel_noise", accel_noise_);
}

void ImuDriven::addStates(StateLayoutBuilder & builder) const
{
  builder.add(input_ + "/gyro_bias", BlockKind::kVector, 3);
  builder.add(input_ + "/accel_bias", BlockKind::kVector, 3);
}

State ImuDriven::predict(const State & x, double dt, const Inputs & u) const
{
  const Input * imu = u.get(input_);
  if (imu == nullptr) {
    return ConstantAcceleration::predict(x, dt, u);
  }
  const Eigen::Vector3d w = imu->u.head<3>() - Eigen::Vector3d(x.vector(input_ + "/gyro_bias"));
  const Eigen::Vector3d up_in_body = x.orientation().conjugate() * Eigen::Vector3d(0.0, 0.0, gravity_);
  const Eigen::Vector3d a = imu->u.tail<3>() - Eigen::Vector3d(x.vector(input_ + "/accel_bias")) - up_in_body;
  State y = x;
  integrateBody(y, x.linearVelocity(), w, a, dt);
  y.vector(blocks::kAngularVelocity) = w;
  y.vector(blocks::kLinearAcceleration) = a;
  return y;
}

Eigen::MatrixXd ImuDriven::processNoise(const State & x, double dt, const Inputs & u) const
{
  Eigen::MatrixXd Q = noise_.covariance(x.layout(), dt);
  const Input * imu = u.get(input_);
  if (imu == nullptr) {
    return Q;
  }
  // The input's own noise (per reading), from its covariance or the parameters
  Eigen::Vector3d gyro = Eigen::Vector3d::Constant(gyro_noise_ * gyro_noise_);
  Eigen::Vector3d accel = Eigen::Vector3d::Constant(accel_noise_ * accel_noise_);
  if (imu->covariance.rows() == 6 && imu->covariance.diagonal().minCoeff() > 0.0) {
    gyro = imu->covariance.diagonal().head<3>();
    accel = imu->covariance.diagonal().tail<3>();
  }
  const StateLayout & layout = x.layout();
  addVariance(Q, layout, blocks::kAngularVelocity, gyro);
  addVariance(Q, layout, blocks::kLinearAcceleration, accel);
  addVariance(Q, layout, blocks::kOrientation, gyro * dt * dt);
  addVariance(Q, layout, blocks::kLinearVelocity, accel * dt * dt);
  return Q;
}

// ---------------------------------------------------------------- kinematic bicycle
void KinematicBicycle::initialize(const Params & params)
{
  noise_.position = {0.02, 0.02, 0.05};
  noise_.orientation = {0.005, 0.005, 0.002};
  noise_.velocity = {0.05, 0.05, 0.05};
  noise_.angular_velocity = {0.3, 0.3, 0.05};
  noise_.read(params);
  input_ = params.getString("input", "");
  if (input_.empty()) {
    throw std::invalid_argument("motion_model.input: the wheel sensor that drives kinematic_bicycle");
  }
  wheel_base_ = params.getDouble("wheel_base", wheel_base_);
  speed_noise_ = params.getDouble("speed_noise", speed_noise_);
  steering_noise_ = params.getDouble("steering_noise", steering_noise_);
}

State KinematicBicycle::predict(const State & x, double dt, const Inputs & u) const
{
  const Input * wheels = u.get(input_);
  if (wheels == nullptr) {
    return ConstantAcceleration::predict(x, dt, u);
  }
  const double speed = wheels->u(0);
  const double yaw_rate = speed * std::tan(wheels->u(1)) / wheel_base_;
  // base_footprint is half a wheel base ahead of the rear axle
  const Eigen::Vector3d v(speed, yaw_rate * wheel_base_ / 2.0, 0.0);
  const Eigen::Vector3d w_state = x.angularVelocity();
  const Eigen::Vector3d w(w_state.x(), w_state.y(), yaw_rate);
  State y = x;
  integrateBody(y, v, w, Eigen::Vector3d::Zero(), dt);
  y.vector(blocks::kLinearVelocity) = v;
  y.vector(blocks::kAngularVelocity) = w;
  return y;
}

Eigen::MatrixXd KinematicBicycle::processNoise(const State & x, double dt, const Inputs & u) const
{
  Eigen::MatrixXd Q = noise_.covariance(x.layout(), dt);
  const Input * wheels = u.get(input_);
  if (wheels == nullptr) {
    return Q;
  }
  const double speed = wheels->u(0);
  const double steering = wheels->u(1);
  const double c = std::cos(steering);
  const double speed_var = speed_noise_ * speed_noise_;
  // Yaw rate v tan(d) / L: from the speed and the steering noise
  const double yaw_rate_var =
    std::pow(std::tan(steering) / wheel_base_, 2) * speed_var +
    std::pow(speed / (wheel_base_ * c * c), 2) * steering_noise_ * steering_noise_;
  const StateLayout & layout = x.layout();
  addVariance(Q, layout, blocks::kLinearVelocity, Eigen::Vector3d(speed_var, speed_var * 0.1, speed_var * 0.1));
  addVariance(Q, layout, blocks::kAngularVelocity, Eigen::Vector3d(0.0, 0.0, yaw_rate_var));
  addVariance(Q, layout, blocks::kOrientation, Eigen::Vector3d(0.0, 0.0, yaw_rate_var * dt * dt));
  addVariance(Q, layout, blocks::kPosition, Eigen::Vector3d::Constant(speed_var * dt * dt));
  return Q;
}

// ---------------------------------------------------------------- dynamic bicycle
void DynamicBicycle::initialize(const Params & params)
{
  noise_.position = {0.02, 0.02, 0.05};
  noise_.orientation = {0.005, 0.005, 0.002};
  noise_.velocity = {0.05, 0.05, 0.05};
  noise_.angular_velocity = {0.3, 0.3, 0.05};
  noise_.read(params);
  input_ = params.getString("input", "");
  if (input_.empty()) {
    throw std::invalid_argument("motion_model.input: the wheel sensor that drives dynamic_bicycle");
  }
  mass_ = params.getDouble("mass", mass_);
  yaw_inertia_ = params.getDouble("yaw_inertia", yaw_inertia_);
  lf_ = params.getDouble("lf", lf_);
  lr_ = params.getDouble("lr", lr_);
  cf_ = params.getDouble("cornering_stiffness_front", cf_);
  cr_ = params.getDouble("cornering_stiffness_rear", cr_);
  kinematic_speed_ = params.getDouble("kinematic_speed", kinematic_speed_);
  dynamic_speed_ = std::max(kinematic_speed_ + 0.1, params.getDouble("dynamic_speed", dynamic_speed_));
  speed_noise_ = params.getDouble("speed_noise", speed_noise_);
  steering_noise_ = params.getDouble("steering_noise", steering_noise_);
  lateral_noise_ = params.getDouble("lateral_noise", lateral_noise_);
}

State DynamicBicycle::predict(const State & x, double dt, const Inputs & u) const
{
  const Input * wheels = u.get(input_);
  if (wheels == nullptr) {
    return ConstantAcceleration::predict(x, dt, u);
  }
  const double speed = wheels->u(0);
  const double steering = wheels->u(1);
  const double wheel_base = lf_ + lr_;
  // base_footprint is in the middle of the wheel base; the centre of mass lr ahead of the
  // rear axle, so at com_x from base_footprint
  const double com_x = lr_ - wheel_base / 2.0;

  // Kinematic: no slip, the rear axle does not move sideways
  const double r_kinematic = speed * std::tan(steering) / wheel_base;
  const double v_kinematic = r_kinematic * lr_;

  double v = v_kinematic;
  double r = r_kinematic;
  const double blend = std::clamp((speed - kinematic_speed_) / (dynamic_speed_ - kinematic_speed_), 0.0, 1.0);
  if (blend > 0.0) {
    // Dynamic, from the state: lateral velocity at the centre of mass and yaw rate
    const Eigen::Vector3d w_state = x.angularVelocity();
    const double r0 = w_state.z();
    const double v0 = x.linearVelocity().y() + r0 * com_x;
    Eigen::Matrix2d A;
    A << -(cf_ + cr_) / (mass_ * speed), (lr_ * cr_ - lf_ * cf_) / (mass_ * speed) - speed,
      (lr_ * cr_ - lf_ * cf_) / (yaw_inertia_ * speed), -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (yaw_inertia_ * speed);
    const Eigen::Vector2d B(cf_ / mass_, lf_ * cf_ / yaw_inertia_);
    // Backward Euler: (I - dt A) x' = x + dt B steering
    const Eigen::Vector2d next =
      (Eigen::Matrix2d::Identity() - dt * A).partialPivLu().solve(Eigen::Vector2d(v0, r0) + dt * B * steering);
    v = blend * next(0) + (1.0 - blend) * v_kinematic;
    r = blend * next(1) + (1.0 - blend) * r_kinematic;
  }

  const Eigen::Vector3d w_state = x.angularVelocity();
  const Eigen::Vector3d body_velocity(speed, v - r * com_x, 0.0);  // back to base_footprint
  const Eigen::Vector3d w(w_state.x(), w_state.y(), r);
  State y = x;
  integrateBody(y, body_velocity, w, Eigen::Vector3d::Zero(), dt);
  y.vector(blocks::kLinearVelocity) = body_velocity;
  y.vector(blocks::kAngularVelocity) = w;
  return y;
}

Eigen::MatrixXd DynamicBicycle::processNoise(const State & x, double dt, const Inputs & u) const
{
  Eigen::MatrixXd Q = noise_.covariance(x.layout(), dt);
  const Input * wheels = u.get(input_);
  if (wheels == nullptr) {
    return Q;
  }
  const double speed = wheels->u(0);
  const double steering = wheels->u(1);
  const double wheel_base = lf_ + lr_;
  const double c = std::cos(steering);
  const double speed_var = speed_noise_ * speed_noise_;
  // The steering and speed noise as for the kinematic model, plus the tyre model's own error
  const double yaw_rate_var =
    std::pow(std::tan(steering) / wheel_base, 2) * speed_var +
    std::pow(speed / (wheel_base * c * c), 2) * steering_noise_ * steering_noise_;
  const double lateral_var = lateral_noise_ * lateral_noise_ * dt;
  const StateLayout & layout = x.layout();
  addVariance(Q, layout, blocks::kLinearVelocity, Eigen::Vector3d(speed_var, lateral_var, speed_var * 0.1));
  addVariance(Q, layout, blocks::kAngularVelocity, Eigen::Vector3d(0.0, 0.0, yaw_rate_var));
  addVariance(Q, layout, blocks::kOrientation, Eigen::Vector3d(0.0, 0.0, yaw_rate_var * dt * dt));
  addVariance(Q, layout, blocks::kPosition, Eigen::Vector3d::Constant(speed_var * dt * dt));
  return Q;
}

}  // namespace sac_localization

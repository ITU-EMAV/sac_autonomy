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
  const std::string speed_from = params.getString("speed_from", "state");
  if (speed_from != "state" && speed_from != "input") {
    throw std::invalid_argument("motion_model.speed_from: state or input");
  }
  speed_from_state_ = speed_from == "state";
  if (!initialized_) {
    const std::string mode = params.getString("cornering_stiffness_mode", "manual");
    const std::string what = params.getString("cornering_stiffness_estimate", "grip");
    if (mode == "manual") {
      estimate_ = Estimate::kNone;
    } else if (mode != "estimate") {
      throw std::invalid_argument("motion_model.cornering_stiffness_mode: manual or estimate");
    } else if (what == "grip") {
      estimate_ = Estimate::kGrip;
    } else if (what == "front_rear") {
      estimate_ = Estimate::kFrontRear;
    } else {
      throw std::invalid_argument("motion_model.cornering_stiffness_estimate: grip or front_rear");
    }
    initialized_ = true;
  }
  stiffness_uncertainty_ = params.getDouble("cornering_stiffness_uncertainty", stiffness_uncertainty_);
  stiffness_noise_ = params.getDouble("cornering_stiffness_noise", stiffness_noise_);
  max_stiffness_factor_ = std::max(1.0, params.getDouble("max_cornering_stiffness_factor", max_stiffness_factor_));
}

void DynamicBicycle::addStates(StateLayoutBuilder & builder) const
{
  if (estimate_ != Estimate::kNone) {
    builder.add(kStiffnessBlock, BlockKind::kVector, estimate_ == Estimate::kGrip ? 1 : 2);
  }
}

void DynamicBicycle::initializeBelief(Belief & belief) const
{
  if (estimate_ == Estimate::kNone) {
    return;
  }
  const BlockInfo & b = belief.state.layout().block(kStiffnessBlock);
  belief.state.vector(kStiffnessBlock).setZero();  // the configured values
  belief.covariance.block(b.tangent_offset, 0, b.tangent_size, belief.covariance.cols()).setZero();
  belief.covariance.block(0, b.tangent_offset, belief.covariance.rows(), b.tangent_size).setZero();
  for (int i = 0; i < b.tangent_size; ++i) {
    belief.covariance(b.tangent_offset + i, b.tangent_offset + i) = stiffness_uncertainty_ * stiffness_uncertainty_;
  }
}

std::vector<EstimatedParameter> DynamicBicycle::estimatedParameters(const Belief & belief) const
{
  if (estimate_ == Estimate::kNone) {
    return {};
  }
  const Eigen::Vector2d c = corneringStiffness(belief.state);
  const BlockInfo & b = belief.state.layout().block(kStiffnessBlock);
  auto stddev = [&](int i) {
    // Of C = C0 exp(s): dC = C ds
    const int k = std::min(i, b.tangent_size - 1);
    return c(i) * std::sqrt(std::max(0.0, belief.covariance(b.tangent_offset + k, b.tangent_offset + k)));
  };
  return {
    {"cornering_stiffness_front", c(0), stddev(0)},
    {"cornering_stiffness_rear", c(1), stddev(1)},
  };
}

Eigen::Vector2d DynamicBicycle::corneringStiffness(const State & x) const
{
  if (estimate_ == Estimate::kNone) {
    return {cf_, cr_};
  }
  const Eigen::VectorXd s = x.vector(kStiffnessBlock);
  const double limit = std::log(max_stiffness_factor_);
  const double front = std::clamp(s(0), -limit, limit);
  const double rear = std::clamp(s(s.size() - 1), -limit, limit);
  return {cf_ * std::exp(front), cr_ * std::exp(rear)};
}

State DynamicBicycle::predict(const State & x, double dt, const Inputs & u) const
{
  const Input * wheels = u.get(input_);
  if (wheels == nullptr) {
    return ConstantAcceleration::predict(x, dt, u);
  }
  const double steering = wheels->u(1);
  // The speed: the state's, moved on with its longitudinal acceleration (the wheel speed is
  // then a measurement, which the filter can reject while a wheel slips or is in the air),
  // or the wheel speed input as it is
  const double speed = speed_from_state_
                         ? x.linearVelocity().x() + x.linearAcceleration().x() * dt
                         : wheels->u(0);
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
    const Eigen::Vector2d c = corneringStiffness(x);
    const double cf = c(0);
    const double cr = c(1);
    Eigen::Matrix2d A;
    A << -(cf + cr) / (mass_ * speed), (lr_ * cr - lf_ * cf) / (mass_ * speed) - speed,
      (lr_ * cr - lf_ * cf) / (yaw_inertia_ * speed), -(lf_ * lf_ * cf + lr_ * lr_ * cr) / (yaw_inertia_ * speed);
    const Eigen::Vector2d B(cf / mass_, lf_ * cf / yaw_inertia_);
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
  // The lateral acceleration the model implies (v' + w x v), so the accelerometers see a
  // turning car rather than a tilted one
  const Eigen::Vector3d old_velocity = x.linearVelocity();
  Eigen::Vector3d acceleration = x.linearAcceleration();
  acceleration.y() = (body_velocity.y() - old_velocity.y()) / std::max(dt, 1e-6) + w.cross(body_velocity).y();
  y.vector(blocks::kLinearVelocity) = body_velocity;
  y.vector(blocks::kAngularVelocity) = w;
  y.vector(blocks::kLinearAcceleration) = acceleration;
  return y;
}

Eigen::MatrixXd DynamicBicycle::processNoise(const State & x, double dt, const Inputs & u) const
{
  Eigen::MatrixXd Q = noise_.covariance(x.layout(), dt);
  if (estimate_ != Estimate::kNone) {
    const BlockInfo & b = x.layout().block(kStiffnessBlock);
    for (int i = 0; i < b.tangent_size; ++i) {
      Q(b.tangent_offset + i, b.tangent_offset + i) = stiffness_noise_ * stiffness_noise_ * dt;
    }
  }
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
  // With the speed from the state its noise is BlockNoise's (velocity and acceleration)
  const double input_speed_var = speed_from_state_ ? 0.0 : speed_var;
  addVariance(Q, layout, blocks::kLinearVelocity, Eigen::Vector3d(input_speed_var, lateral_var, speed_var * 0.1));
  addVariance(Q, layout, blocks::kAngularVelocity, Eigen::Vector3d(0.0, 0.0, yaw_rate_var));
  addVariance(Q, layout, blocks::kOrientation, Eigen::Vector3d(0.0, 0.0, yaw_rate_var * dt * dt));
  addVariance(Q, layout, blocks::kPosition, Eigen::Vector3d::Constant(speed_var * dt * dt));
  return Q;
}

}  // namespace sac_localization

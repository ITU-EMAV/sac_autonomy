#include "sac_localization_adapters/adapters.hpp"

#include <chrono>
#include <cmath>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "sac_localization/core/measurement_models.hpp"
#include "sac_localization/core/so3.hpp"

namespace sac_localization
{

namespace
{
using RowMatrix3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

Eigen::Matrix3d matrix3(const std::array<double, 9> & values)
{
  return Eigen::Map<const RowMatrix3d>(values.data());
}

/// 3x3 block (row, col in 0/1) of a row-major 6x6 ROS covariance
Eigen::Matrix3d block6(const std::array<double, 36> & values, int row, int col)
{
  const Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> m(values.data());
  return m.block<3, 3>(3 * row, 3 * col);
}

Eigen::Vector3d vector3(const geometry_msgs::msg::Vector3 & v) { return {v.x, v.y, v.z}; }

/// Collects parts of one measurement: models, values, noise; fused together.
struct Parts
{
  std::vector<std::shared_ptr<const MeasurementModel>> models;
  std::vector<Eigen::VectorXd> values;
  std::vector<Eigen::MatrixXd> noises;

  void add(std::shared_ptr<const MeasurementModel> model, Eigen::VectorXd z, Eigen::MatrixXd R)
  {
    models.push_back(std::move(model));
    values.push_back(std::move(z));
    noises.push_back(std::move(R));
  }
  bool empty() const { return models.empty(); }

  Measurement build(Stamp stamp) const
  {
    Measurement m;
    m.stamp = stamp;
    int dim = 0;
    for (const auto & z : values) {
      dim += static_cast<int>(z.size());
    }
    m.z.resize(dim);
    m.R = Eigen::MatrixXd::Zero(dim, dim);
    int row = 0;
    for (std::size_t i = 0; i < values.size(); ++i) {
      const int d = static_cast<int>(values[i].size());
      m.z.segment(row, d) = values[i];
      m.R.block(row, row, d, d) = noises[i];
      row += d;
    }
    m.model = models.size() == 1 ? models.front() : std::make_shared<StackedModel>(models);
    return m;
  }
};

/// Selects components of a 3D model and of its noise
void addSubset(
  Parts & parts, const std::shared_ptr<const MeasurementModel> & model, const Eigen::Vector3d & z,
  const Eigen::Matrix3d & R, const std::vector<int> & components)
{
  if (components.size() == 3) {
    parts.add(model, z, R);
    return;
  }
  Eigen::VectorXd zs(components.size());
  Eigen::MatrixXd Rs(components.size(), components.size());
  for (std::size_t i = 0; i < components.size(); ++i) {
    zs(i) = z(components[i]);
    for (std::size_t j = 0; j < components.size(); ++j) {
      Rs(i, j) = R(components[i], components[j]);
    }
  }
  parts.add(std::make_shared<SubsetModel>(model, components), zs, Rs);
}

double meanOf(const sensor_msgs::msg::JointState & m, const std::vector<std::string> & joints, bool velocity, bool & found)
{
  double sum = 0.0;
  int count = 0;
  const auto & values = velocity ? m.velocity : m.position;
  for (const std::string & joint : joints) {
    for (std::size_t i = 0; i < m.name.size() && i < values.size(); ++i) {
      if (m.name[i] == joint) {
        sum += values[i];
        ++count;
      }
    }
  }
  found = count > 0;
  return found ? sum / count : 0.0;
}
}  // namespace

// ---------------------------------------------------------------- IMU watch
void ImuWatch::subscribe(rclcpp::Node * node, const std::string & topic)
{
  subscription_ = node->create_subscription<sensor_msgs::msg::Imu>(
    topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
      has_ = true;
      stamp_ = static_cast<Stamp>(m->header.stamp.sec) * 1000000000LL + m->header.stamp.nanosec;
      acceleration_ = vector3(m->linear_acceleration);
      angular_velocity_ = vector3(m->angular_velocity);
    });
}

// ---------------------------------------------------------------- imu
void ImuAdapter::configure(const Params & params)
{
  estimate_biases_ = params.getBool("estimate_biases", true);
}

void ImuAdapter::addStates(StateLayoutBuilder & builder) const
{
  if (!estimate_biases_) {
    return;
  }
  if (!asInput() && uses("angular_velocity")) {
    builder.add(name_ + "/gyro_bias", BlockKind::kVector, 3);
  }
  if (!asInput() && uses("linear_acceleration")) {
    builder.add(name_ + "/accel_bias", BlockKind::kVector, 3);
  }
}

void ImuAdapter::convert(const sensor_msgs::msg::Imu & m)
{
  const auto mnt = mount(m.header.frame_id);
  if (!mnt) {
    return;
  }
  const Eigen::Quaterniond mount_rotation(mnt->linear());
  const Eigen::Vector3d gyro = vector3(m.angular_velocity);
  const Eigen::Vector3d accel = vector3(m.linear_acceleration);
  const auto gyro_noise = noise("angular_velocity_covariance", matrix3(m.angular_velocity_covariance));
  const auto accel_noise = noise("linear_acceleration_covariance", matrix3(m.linear_acceleration_covariance));

  if (asInput()) {
    // Rotated to the body frame for the imu_driven model
    const Eigen::Matrix3d R = mnt->linear();
    Input input;
    input.stamp = stamp(m.header.stamp);
    input.u.resize(6);
    input.u << R * gyro, R * accel;
    input.covariance = Eigen::MatrixXd::Zero(6, 6);
    if (gyro_noise && accel_noise) {
      input.covariance.block<3, 3>(0, 0) = R * *gyro_noise * R.transpose();
      input.covariance.block<3, 3>(3, 3) = R * *accel_noise * R.transpose();
    }
    send(std::move(input));
    return;
  }

  // Separate measurements, each with its own rejection threshold: an impact makes the
  // accelerometer spike (reject it) while the gyro still measures the real rotation (keep it)
  const Stamp t = stamp(m.header.stamp);
  if (uses("angular_velocity") && gyro_noise) {
    Parts parts;
    parts.add(
      std::make_shared<AngularVelocityModel>(mount_rotation, estimate_biases_ ? name_ + "/gyro_bias" : ""), gyro,
      *gyro_noise);
    send(parts.build(t), "angular_velocity_rejection_threshold");
  }
  if (uses("linear_acceleration") && accel_noise) {
    Parts parts;
    parts.add(
      std::make_shared<SpecificForceModel>(*mnt, estimate_biases_ ? name_ + "/accel_bias" : ""), accel, *accel_noise);
    send(parts.build(t), "linear_acceleration_rejection_threshold");
  }
  if (uses("orientation") && m.orientation_covariance[0] >= 0.0) {
    const auto orientation_noise = noise("orientation_covariance", matrix3(m.orientation_covariance));
    if (orientation_noise) {
      const Eigen::Quaterniond q(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z);
      Parts parts;
      parts.add(std::make_shared<OrientationModel>(mount_rotation), logSO3(q), *orientation_noise);
      send(parts.build(t), "orientation_rejection_threshold");
    }
  }
}

// ---------------------------------------------------------------- gnss position
void GnssPositionAdapter::configure(const Params & params)
{
  if (context().map == nullptr) {
    throw std::invalid_argument("sensors." + name_ + ": gnss_position needs a datum in the config");
  }
  min_status_ = params.getInt("min_status", sensor_msgs::msg::NavSatStatus::STATUS_FIX);
}

void GnssPositionAdapter::convert(const sensor_msgs::msg::NavSatFix & m)
{
  if (m.status.status < min_status_ || !std::isfinite(m.latitude) || !std::isfinite(m.longitude) ||
      !std::isfinite(m.altitude))
  {
    return;
  }
  const auto mnt = mount(m.header.frame_id);
  if (!mnt) {
    return;
  }
  const MapFrame & map = *context().map;
  const Eigen::Vector3d z = map.toMap(m.latitude, m.longitude, m.altitude);
  Eigen::Matrix3d from_message = Eigen::Matrix3d::Zero();
  if (m.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    from_message = map.mapFromEnu() * matrix3(m.position_covariance) * map.mapFromEnu().transpose();
  }
  auto R = noise("covariance", from_message);
  if (!R) {
    return;
  }
  if (from_message.isZero()) {
    // Config variances are east, north, up: turn them into the map frame
    *R = map.mapFromEnu() * *R * map.mapFromEnu().transpose();
  }
  std::vector<int> components;
  if (uses("horizontal")) {
    components = {0, 1};
  }
  if (uses("vertical")) {
    components.push_back(2);
  }
  if (components.empty()) {
    return;
  }
  Parts parts;
  addSubset(parts, std::make_shared<PositionModel>(mnt->translation()), z, *R, components);
  send(parts.build(stamp(m.header.stamp)));
}

// ---------------------------------------------------------------- wheels
void WheelAdapter::configure(const Params & params)
{
  speed_joints_ = params.getStrings("speed_joints", {"rear_left_wheel_joint", "rear_right_wheel_joint"});
  steering_joints_ = params.getStrings(
    "steering_joints", {"front_left_wheel_steering_joint", "front_right_wheel_steering_joint"});
  wheel_radius_ = params.getDouble("wheel_radius", wheel_radius_);
  wheel_base_ = params.getDouble("wheel_base", wheel_base_);
  const auto rear = params.getDoubles("rear_axle", {-wheel_base_ / 2.0, 0.0, 0.0});
  rear_axle_ = Eigen::Vector3d(rear.at(0), rear.at(1), rear.at(2));
  const std::string imu = params.getString("slip_check_imu", "");
  if (!imu.empty()) {
    imu_.subscribe(context().node, imu);
  }
}

bool WheelAdapter::slipping(Stamp t, double speed)
{
  if (last_stamp_ != 0 && t > last_stamp_) {
    const double a = (speed - last_speed_) / toSeconds(t - last_stamp_);
    wheel_acceleration_ = 0.7 * wheel_acceleration_ + 0.3 * a;  // smoothed over a few readings
  }
  last_stamp_ = t;
  last_speed_ = speed;
  if (!imu_.fresh(t)) {
    return false;
  }
  // The wheels speed up or slow down faster than the car: they spin or lock (hold 0.3 s)
  if (std::abs(wheel_acceleration_ - imu_.acceleration().x()) > params().getDouble("max_slip_acceleration", 2.5)) {
    slip_until_ = t + fromSeconds(0.3);
  }
  return t < slip_until_;
}

void WheelAdapter::convert(const sensor_msgs::msg::JointState & m)
{
  bool has_speed = false;
  bool has_steering = false;
  const double speed = meanOf(m, speed_joints_, true, has_speed) * wheel_radius_;
  const double steering = meanOf(m, steering_joints_, false, has_steering);
  if (!has_speed) {
    return;  // another JointState on the same topic
  }
  const Stamp t = stamp(m.header.stamp);
  const bool slip = imu_.active() && slipping(t, speed);
  if (asInput()) {
    Input input;
    input.stamp = t;
    input.u = Eigen::Vector2d(speed, steering);
    input.covariance = Eigen::MatrixXd::Zero(2, 2);
    send(std::move(input));
    // Also as measurements (e.g. the speed, for a model that only takes the steering as
    // input: a slipping or airborne wheel's speed can then be rejected)
    if (!params().getBool("also_measure", false)) {
      return;
    }
  }
  if (slip) {
    return;  // the wheels do not tell the car's speed now
  }
  const std::vector<double> variances = params().getDoubles("covariance", {0.01, 0.001});
  Eigen::Isometry3d rear = Eigen::Isometry3d::Identity();
  rear.translation() = rear_axle_;
  Parts parts;
  if (uses("speed")) {
    parts.add(
      std::make_shared<SubsetModel>(std::make_shared<BodyVelocityModel>(rear), std::vector<int>{0}),
      Eigen::VectorXd::Constant(1, speed), Eigen::MatrixXd::Constant(1, 1, variances.at(0)));
  }
  if (uses("yaw_rate") && has_steering) {
    parts.add(
      std::make_shared<SubsetModel>(
        std::make_shared<AngularVelocityModel>(Eigen::Quaterniond::Identity()), std::vector<int>{2}),
      Eigen::VectorXd::Constant(1, speed * std::tan(steering) / wheel_base_),
      Eigen::MatrixXd::Constant(1, 1, variances.at(1)));
  }
  if (!parts.empty()) {
    send(parts.build(t));
  }
}

// ---------------------------------------------------------------- zero velocity
void ZeroVelocityAdapter::configure(const Params & params)
{
  speed_joints_ = params.getStrings("speed_joints", {"rear_left_wheel_joint", "rear_right_wheel_joint"});
  wheel_radius_ = params.getDouble("wheel_radius", wheel_radius_);
  const std::string imu = params.getString("imu_topic", "");
  if (!imu.empty()) {
    imu_.subscribe(context().node, imu);
  }
}

void ZeroVelocityAdapter::convert(const sensor_msgs::msg::JointState & m)
{
  bool found = false;
  const double speed = meanOf(m, speed_joints_, true, found) * wheel_radius_;
  if (!found || std::abs(speed) > params().getDouble("threshold", 0.02)) {
    return;
  }
  if (imu_.active()) {
    // Standing still only if the IMU agrees: locked wheels of a sliding car read 0 too
    if (!imu_.fresh(stamp(m.header.stamp)) ||
        std::abs(imu_.acceleration().norm() - 9.80665) > params().getDouble("max_acceleration_deviation", 0.3) ||
        imu_.angularVelocity().norm() > params().getDouble("max_angular_velocity", 0.05))
    {
      return;
    }
  }
  const std::vector<double> variances = params().getDoubles("covariance", {1e-4, 1e-6});
  Parts parts;
  parts.add(
    std::make_shared<BodyVelocityModel>(Eigen::Isometry3d::Identity()), Eigen::Vector3d::Zero(),
    Eigen::Matrix3d::Identity() * variances.at(0));
  parts.add(
    std::make_shared<AngularVelocityModel>(Eigen::Quaterniond::Identity()), Eigen::Vector3d::Zero(),
    Eigen::Matrix3d::Identity() * variances.at(1));
  send(parts.build(stamp(m.header.stamp)));
}

// ---------------------------------------------------------------- nonholonomic
void NonholonomicAdapter::initialize(
  const AdapterContext & context, const std::string & name, std::shared_ptr<const Params> params)
{
  context_ = context;
  name_ = name;
  params_ = std::move(params);
  const auto lever = params_->getDoubles("lever_arm", {-1.873 / 2.0, 0.0, 0.0});
  lever_arm_ = Eigen::Vector3d(lever.at(0), lever.at(1), lever.at(2));
}

void NonholonomicAdapter::start(std::shared_ptr<const StateLayout>)
{
  if (!params_->getBool("enabled", true)) {
    return;  // e.g. with a motion model that models the slip itself
  }
  const double rate = params_->getDouble("rate", 20.0);
  timer_ = rclcpp::create_timer(
    context_.node, context_.node->get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() { tick(); });
}

void NonholonomicAdapter::tick()
{
  ++statistics_.received;
  const std::vector<double> variances = params_->getDoubles("covariance", {0.01, 0.01});
  Eigen::Isometry3d point = Eigen::Isometry3d::Identity();
  point.translation() = lever_arm_;
  Measurement m;
  m.stamp = context_.node->now().nanoseconds();
  m.source = name_;
  m.model = std::make_shared<SubsetModel>(std::make_shared<BodyVelocityModel>(point), std::vector<int>{1, 2});
  m.z = Eigen::Vector2d::Zero();
  m.R = Eigen::Vector2d(variances.at(0), variances.at(1)).asDiagonal();
  m.rejection_threshold = params_->getDouble("rejection_threshold", 10.0);
  context_.sink->add(std::move(m));
}

// ---------------------------------------------------------------- odometry, twist, pose
void OdometryAdapter::configure(const Params & params)
{
  velocity_from_pose_ = params.getBool("velocity_from_pose", velocity_from_pose_);
}

void OdometryAdapter::convert(const nav_msgs::msg::Odometry & m)
{
  if (velocity_from_pose_) {
    // How it moved since the last pose, in the body frame, as a twist at the middle
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
    pose.linear() = Eigen::Quaterniond(
      m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z)
      .normalized().toRotationMatrix();
    const Stamp now = stamp(m.header.stamp);
    const bool first = !last_pose_;
    const double dt = first ? 0.0 : toSeconds(now - last_stamp_);
    if (!first && dt < 0.02) {
      return;
    }
    const Eigen::Isometry3d previous = first ? pose : *last_pose_;
    last_pose_ = pose;
    last_stamp_ = now;
    if (first || dt > 0.5) {
      return;  // nothing to compare with, or a gap
    }
    const Eigen::Isometry3d delta = previous.inverse() * pose;
    const Eigen::Vector3d v = delta.translation() / dt;
    const Eigen::Vector3d w = logSO3(Eigen::Quaterniond(delta.linear())) / dt;
    const Stamp middle = now - static_cast<Stamp>(dt * 0.5e9);
    Parts parts;
    if (const auto mnt = mount(m.child_frame_id)) {
      if (uses("linear_velocity")) {
        if (auto R = noise("linear_velocity_covariance", block6(m.twist.covariance, 0, 0))) {
          parts.add(std::make_shared<BodyVelocityModel>(*mnt), v, *R);
        }
      }
      if (uses("angular_velocity")) {
        if (auto R = noise("angular_velocity_covariance", block6(m.twist.covariance, 1, 1))) {
          parts.add(std::make_shared<AngularVelocityModel>(Eigen::Quaterniond(mnt->linear())), w, *R);
        }
      }
    }
    if (!parts.empty()) {
      send(parts.build(middle));
    }
    return;
  }
  Parts parts;
  if (uses("linear_velocity") || uses("angular_velocity")) {
    if (const auto mnt = mount(m.child_frame_id)) {
      if (uses("linear_velocity")) {
        if (auto R = noise("linear_velocity_covariance", block6(m.twist.covariance, 0, 0))) {
          parts.add(std::make_shared<BodyVelocityModel>(*mnt), vector3(m.twist.twist.linear), *R);
        }
      }
      if (uses("angular_velocity")) {
        if (auto R = noise("angular_velocity_covariance", block6(m.twist.covariance, 1, 1))) {
          parts.add(
            std::make_shared<AngularVelocityModel>(Eigen::Quaterniond(mnt->linear())),
            vector3(m.twist.twist.angular), *R);
        }
      }
    }
  }
  if ((uses("position") || uses("orientation")) && m.header.frame_id == context().world_frame) {
    if (const auto mnt = mount(m.child_frame_id)) {
      const Eigen::Vector3d p(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
      const Eigen::Quaterniond q(
        m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z);
      if (uses("position")) {
        if (auto R = noise("position_covariance", block6(m.pose.covariance, 0, 0))) {
          parts.add(std::make_shared<PositionModel>(mnt->translation()), p, *R);
        }
      }
      if (uses("orientation")) {
        if (auto R = noise("orientation_covariance", block6(m.pose.covariance, 1, 1))) {
          parts.add(std::make_shared<OrientationModel>(Eigen::Quaterniond(mnt->linear())), logSO3(q), *R);
        }
      }
    }
  }
  if (!parts.empty()) {
    send(parts.build(stamp(m.header.stamp)));
  }
}

void TwistAdapter::convert(const geometry_msgs::msg::TwistWithCovarianceStamped & m)
{
  const auto mnt = mount(m.header.frame_id);
  if (!mnt) {
    return;
  }
  Parts parts;
  if (uses("linear_velocity")) {
    if (auto R = noise("linear_velocity_covariance", block6(m.twist.covariance, 0, 0))) {
      parts.add(std::make_shared<BodyVelocityModel>(*mnt), vector3(m.twist.twist.linear), *R);
    }
  }
  if (uses("angular_velocity")) {
    if (auto R = noise("angular_velocity_covariance", block6(m.twist.covariance, 1, 1))) {
      parts.add(
        std::make_shared<AngularVelocityModel>(Eigen::Quaterniond(mnt->linear())), vector3(m.twist.twist.angular),
        *R);
    }
  }
  if (!parts.empty()) {
    send(parts.build(stamp(m.header.stamp)));
  }
}

void PoseAdapter::convert(const geometry_msgs::msg::PoseWithCovarianceStamped & m)
{
  if (m.header.frame_id != context().world_frame) {
    RCLCPP_WARN_THROTTLE(
      context().node->get_logger(), *context().node->get_clock(), 5000, "%s: pose in %s, expected %s",
      name_.c_str(), m.header.frame_id.c_str(), context().world_frame.c_str());
    return;
  }
  const auto mnt = mount("");  // the `frame` parameter, or the base frame
  if (!mnt) {
    return;
  }
  Parts parts;
  if (uses("position")) {
    if (auto R = noise("position_covariance", block6(m.pose.covariance, 0, 0))) {
      const Eigen::Vector3d p(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
      parts.add(std::make_shared<PositionModel>(mnt->translation()), p, *R);
    }
  }
  if (uses("orientation")) {
    if (auto R = noise("orientation_covariance", block6(m.pose.covariance, 1, 1))) {
      const Eigen::Quaterniond q(
        m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z);
      parts.add(std::make_shared<OrientationModel>(Eigen::Quaterniond(mnt->linear())), logSO3(q), *R);
    }
  }
  if (!parts.empty()) {
    send(parts.build(stamp(m.header.stamp)));
  }
}

}  // namespace sac_localization

PLUGINLIB_EXPORT_CLASS(sac_localization::ImuAdapter, sac_localization::SensorAdapter)
PLUGINLIB_EXPORT_CLASS(sac_localization::GnssPositionAdapter, sac_localization::SensorAdapter)
PLUGINLIB_EXPORT_CLASS(sac_localization::WheelAdapter, sac_localization::SensorAdapter)
PLUGINLIB_EXPORT_CLASS(sac_localization::ZeroVelocityAdapter, sac_localization::SensorAdapter)
PLUGINLIB_EXPORT_CLASS(sac_localization::NonholonomicAdapter, sac_localization::SensorAdapter)
PLUGINLIB_EXPORT_CLASS(sac_localization::OdometryAdapter, sac_localization::SensorAdapter)
PLUGINLIB_EXPORT_CLASS(sac_localization::TwistAdapter, sac_localization::SensorAdapter)
PLUGINLIB_EXPORT_CLASS(sac_localization::PoseAdapter, sac_localization::SensorAdapter)

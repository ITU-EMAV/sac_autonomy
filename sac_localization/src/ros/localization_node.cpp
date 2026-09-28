#include "sac_localization/ros/localization_node.hpp"

#include <chrono>
#include <cmath>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "sac_localization/core/measurement_models.hpp"
#include "sac_localization/core/so3.hpp"

namespace sac_localization
{

namespace
{
rclcpp::NodeOptions withParameters(const rclcpp::NodeOptions & options)
{
  rclcpp::NodeOptions o(options);
  o.allow_undeclared_parameters(true);
  o.automatically_declare_parameters_from_overrides(true);
  return o;
}

bool endsWith(const std::string & text, const std::string & suffix)
{
  return text.size() >= suffix.size() && text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("localization", withParameters(options)),
  root_(this, ""),
  estimator_loader_("sac_localization", "sac_localization::Estimator"),
  motion_model_loader_("sac_localization", "sac_localization::MotionModel"),
  adapter_loader_("sac_localization", "sac_localization::SensorAdapter")
{
  world_frame_ = root_.getString("world_frame", "map");
  base_frame_ = root_.getString("base_frame", "base_footprint");
  if (root_.has("datum.latitude")) {
    Datum datum;
    datum.latitude = root_.getDouble("datum.latitude", 0.0);
    datum.longitude = root_.getDouble("datum.longitude", 0.0);
    datum.altitude = root_.getDouble("datum.altitude", 0.0);
    datum.heading = root_.getDouble("datum.heading", 0.0);
    map_.emplace(datum);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Engine and motion model
  const RosParams estimator_params(this, "estimator.");
  estimator_type_ = estimator_params.getString("type", "ekf");
  estimator_ = estimator_loader_.createSharedInstance(estimator_type_);
  estimator_->initialize(estimator_params);
  motion_model_params_ = std::make_shared<RosParams>(this, "motion_model.");
  motion_model_type_ = motion_model_params_->getString("type", "constant_acceleration");
  motion_model_ = motion_model_loader_.createSharedInstance(motion_model_type_);
  motion_model_->initialize(*motion_model_params_);

  // Sensors
  AdapterContext context;
  context.node = this;
  context.tf = tf_buffer_;
  context.base_frame = base_frame_;
  context.world_frame = world_frame_;
  context.map = map_ ? &*map_ : nullptr;
  context.sink = this;
  for (const std::string & name : root_.getStrings("sensors.names", {})) {
    auto params = std::make_shared<RosParams>(this, "sensors." + name + ".");
    const std::string type = params->getString("type", "");
    if (type.empty()) {
      throw std::invalid_argument("sensors." + name + ".type is missing");
    }
    auto adapter = adapter_loader_.createSharedInstance(type);
    adapter->setType(type);
    adapter->initialize(context, name, params);
    adapters_.push_back(adapter);
    adapters_by_name_[name] = adapter;
  }

  // State, fuser, outputs
  StateLayoutBuilder builder;
  motion_model_->addStates(builder);
  for (const auto & adapter : adapters_) {
    adapter->addStates(builder);
  }
  layout_ = builder.build();
  FuserOptions fuser_options;
  fuser_options.history = root_.getDouble("history", 1.0);
  fuser_options.max_prediction_step = root_.getDouble("max_prediction_step", 0.01);
  fuser_ = std::make_unique<Fuser>(layout_, estimator_, motion_model_, fuser_options);
  for (const auto & adapter : adapters_) {
    adapter->start(layout_);
  }
  outputs_ = makeOutputs(*this, context, root_);

  pose_from_ = root_.getString("initial_state.pose_from", "origin");
  const std::string initial_pose_topic = root_.getString("initial_pose_topic", "");
  if (!initial_pose_topic.empty()) {
    initial_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic, 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) { onInitialPose(*m); });
  }
  reset_service_ = create_service<std_srvs::srv::Trigger>(
    "~/reset", [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      restart();
      response->success = true;
      response->message = "restarting from " + pose_from_;
    });
  parameters_callback_ = add_post_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & changed) {
    for (const auto & p : changed) {
      if (p.get_name().rfind("motion_model.", 0) == 0) {
        reinitialize_motion_model_ = true;
      }
    }
  });

  const double frequency = root_.getDouble("frequency", 50.0);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / frequency), [this]() { tick(); });

  RCLCPP_INFO(
    get_logger(), "%s -> %s: %s, %s, %zu sensors, state of %d, starting from %s", world_frame_.c_str(),
    base_frame_.c_str(), estimator_type_.c_str(), motion_model_type_.c_str(), adapters_.size(),
    layout_->tangentSize(), pose_from_.c_str());
}

void LocalizationNode::add(Measurement measurement)
{
  if (fuser_->initialized()) {
    fuser_->add(std::move(measurement));
    return;
  }
  // Before the start: collect GNSS positions for the initial pose
  auto position = std::dynamic_pointer_cast<const PositionModel>(measurement.model);
  if (pose_from_ == "gnss" && position && measurement.z.size() == 3) {
    GnssAverage & g = gnss_[measurement.source];
    g.sum += measurement.z;
    g.lever_arm = position->leverArm();
    ++g.count;
    if (!gnss_since_) {
      gnss_since_ = measurement.stamp;
    }
  }
}

void LocalizationNode::add(Input input)
{
  fuser_->add(std::move(input));
}

Belief LocalizationNode::beliefAt(
  Stamp stamp, const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation, double yaw_variance) const
{
  Belief b{stamp, State(layout_), Eigen::MatrixXd::Zero(layout_->tangentSize(), layout_->tangentSize())};
  b.state.vector(blocks::kPosition) = position;
  b.state.rotation(blocks::kOrientation) = orientation;
  const RosParams p(const_cast<LocalizationNode *>(this), "initial_covariance.");
  auto set = [&b](int offset, int size, double variance) {
    for (int i = 0; i < size; ++i) {
      b.covariance(offset + i, offset + i) = variance;
    }
  };
  for (const BlockInfo & block : layout_->blocks()) {
    double variance = p.getDouble("other", 0.01);
    if (block.name == blocks::kPosition) {
      variance = p.getDouble("position", 1.0);
    } else if (block.name == blocks::kOrientation) {
      set(block.tangent_offset, 2, p.getDouble("roll_pitch", 0.01));
      set(block.tangent_offset + 2, 1, yaw_variance);
      continue;
    } else if (block.name == blocks::kLinearVelocity) {
      variance = p.getDouble("linear_velocity", 0.25);
    } else if (block.name == blocks::kAngularVelocity) {
      variance = p.getDouble("angular_velocity", 0.01);
    } else if (block.name == blocks::kLinearAcceleration) {
      variance = p.getDouble("linear_acceleration", 1.0);
    } else if (endsWith(block.name, "gyro_bias")) {
      variance = p.getDouble("gyro_bias", 1e-4);
    } else if (endsWith(block.name, "accel_bias")) {
      variance = p.getDouble("accel_bias", 0.01);
    }
    set(block.tangent_offset, block.tangent_size, variance);
  }
  motion_model_->initializeBelief(b);
  return b;
}

std::optional<Belief> LocalizationNode::initialBelief(Stamp now)
{
  const double yaw_variance = root_.getDouble("initial_covariance.yaw", 0.05);
  if (initial_pose_) {
    Eigen::Isometry3d pose;
    tf2::fromMsg(initial_pose_->pose.pose, pose);
    Belief b = beliefAt(now, pose.translation(), Eigen::Quaterniond(pose.linear()), yaw_variance);
    const double cov_yaw = initial_pose_->pose.covariance[35];
    if (cov_yaw > 0.0) {
      b.covariance(layout_->block(blocks::kOrientation).tangent_offset + 2,
                   layout_->block(blocks::kOrientation).tangent_offset + 2) = cov_yaw;
    }
    initial_pose_.reset();
    return b;
  }
  if (pose_from_ == "origin") {
    return beliefAt(now, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), yaw_variance);
  }
  if (pose_from_ == "config") {
    const auto p = root_.getDoubles("initial_state.position", {0.0, 0.0, 0.0});
    const double yaw = root_.getDouble("initial_state.yaw", 0.0);
    return beliefAt(
      now, Eigen::Vector3d(p.at(0), p.at(1), p.at(2)),
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())), yaw_variance);
  }
  if (pose_from_ == "gnss") {
    const double duration = root_.getDouble("initial_state.gnss_duration", 2.0);
    if (!gnss_since_ || toSeconds(now - *gnss_since_) < duration) {
      return std::nullopt;
    }
    std::vector<const GnssAverage *> antennas;
    for (const auto & [name, g] : gnss_) {
      if (g.count >= 3) {
        antennas.push_back(&g);
      }
    }
    if (antennas.empty()) {
      return std::nullopt;
    }
    double yaw = root_.getDouble("initial_state.yaw", 0.0);
    double variance = root_.getDouble("initial_covariance.yaw_unknown", 4.0);
    if (antennas.size() >= 2) {
      // Yaw of the baseline in the map minus its yaw on the car
      const Eigen::Vector3d map_baseline = antennas[0]->sum / antennas[0]->count - antennas[1]->sum / antennas[1]->count;
      const Eigen::Vector3d car_baseline = antennas[0]->lever_arm - antennas[1]->lever_arm;
      yaw = wrapAngle(
        std::atan2(map_baseline.y(), map_baseline.x()) - std::atan2(car_baseline.y(), car_baseline.x()));
      variance = yaw_variance;
    }
    const Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    for (const GnssAverage * g : antennas) {
      position += g->sum / g->count - q * g->lever_arm;
    }
    position /= static_cast<double>(antennas.size());
    RCLCPP_INFO(
      get_logger(), "Start from GNSS (%zu antennas): x %.2f y %.2f z %.2f yaw %.1f deg", antennas.size(),
      position.x(), position.y(), position.z(), yaw * 180.0 / M_PI);
    return beliefAt(now, position, q, variance);
  }
  return std::nullopt;  // initial_pose: wait for it
}

void LocalizationNode::restart()
{
  fuser_->clear();
  gnss_.clear();
  gnss_since_.reset();
}

void LocalizationNode::onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  if (!pose.header.frame_id.empty() && pose.header.frame_id != world_frame_) {
    RCLCPP_WARN(
      get_logger(), "Initial pose in %s ignored: expected %s", pose.header.frame_id.c_str(), world_frame_.c_str());
    return;
  }
  restart();
  initial_pose_ = pose;
}

void LocalizationNode::recoverIfLost()
{
  for (const auto & adapter : adapters_) {
    const RosParams p(this, "sensors." + adapter->name() + ".");
    const int limit = p.getInt("max_rejections_in_a_row", 0);
    if (limit <= 0 || adapter->statistics().rejected_in_a_row < static_cast<std::uint64_t>(limit)) {
      continue;
    }
    // A reliable absolute sensor keeps disagreeing: the estimate is more likely lost than
    // the sensor wrong. Widen the uncertainty so its next measurements are taken again.
    const double position = root_.getDouble("recovery_covariance.position", 4.0);
    const double yaw = root_.getDouble("recovery_covariance.yaw", 0.25);
    fuser_->widen(blocks::kPosition, Eigen::Vector3d::Constant(position));
    fuser_->widen(blocks::kOrientation, Eigen::Vector3d(0.0, 0.0, yaw));
    fuser_->widen(blocks::kLinearVelocity, Eigen::Vector3d::Constant(1.0));
    RCLCPP_WARN(
      get_logger(), "%s rejected %d times in a row: widening the position and yaw uncertainty to recover",
      adapter->name().c_str(), limit);
    for (const auto & other : adapters_) {
      other->clearRejectionsInARow();
    }
    return;
  }
}

void LocalizationNode::tick()
{
  const Stamp now = this->now().nanoseconds();
  if (reinitialize_motion_model_) {
    reinitialize_motion_model_ = false;
    motion_model_->initialize(*motion_model_params_);
  }
  if (!fuser_->initialized()) {
    auto initial = initialBelief(now);
    if (!initial) {
      return;
    }
    fuser_->reset(*initial);
  }

  const std::vector<UpdateResult> results = fuser_->update(now);
  if (!fuser_->initialized()) {
    RCLCPP_WARN(get_logger(), "The clock jumped back: restarting");
    tf_buffer_->clear();
    restart();
    return;
  }
  for (const UpdateResult & r : results) {
    auto adapter = adapters_by_name_.find(r.source);
    if (adapter != adapters_by_name_.end()) {
      adapter->second->onResult(r);
    }
  }
  recoverIfLost();
  const OutputContext context{
    fuser_->belief(), results, adapters_, estimator_type_, motion_model_type_,
    motion_model_->estimatedParameters(fuser_->belief())};
  for (const auto & output : outputs_) {
    output->publish(context);
  }
}

}  // namespace sac_localization

RCLCPP_COMPONENTS_REGISTER_NODE(sac_localization::LocalizationNode)

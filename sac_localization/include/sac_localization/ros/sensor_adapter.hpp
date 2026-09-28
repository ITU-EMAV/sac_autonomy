// Sensor adapter: turns a ROS message into measurements and inputs. A plugin (pluginlib,
// base class sac_localization::SensorAdapter); the standard ones are in
// sac_localization_adapters. One adapter instance per sensor in the config:
//
//   sensors:
//     names: [middle_imu, gnss_front_right]
//     middle_imu:
//       type: imu                     # plugin (lookup name in the adapters' plugins.xml)
//       topic: /sac/sensors/middle_imu/imu
//       ...                           # the adapter's parameters
//
// Adapters know ROS and message types; they do not know the estimator. They read the
// sensor's mounting from TF and build measurements from the models in
// core/measurement_models.hpp.

#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>

#include "sac_localization/core/geodesy.hpp"
#include "sac_localization/core/measurement.hpp"
#include "sac_localization/core/params.hpp"
#include "sac_localization/core/state.hpp"

namespace sac_localization
{

/// Where adapters send their measurements and inputs (the node forwards them to the fuser).
class MeasurementSink
{
public:
  virtual ~MeasurementSink() = default;
  virtual void add(Measurement measurement) = 0;
  virtual void add(Input input) = 0;
};

/// Everything an adapter may use.
struct AdapterContext
{
  rclcpp::Node * node = nullptr;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::string base_frame;          // base_footprint
  std::string world_frame;         // map (global filter) or odom (local filter)
  const MapFrame * map = nullptr;  // null when the config has no datum
  MeasurementSink * sink = nullptr;
};

/// Counters for the status output.
struct SensorStatistics
{
  std::uint64_t received = 0;
  std::uint64_t accepted = 0;
  std::uint64_t rejected = 0;
  std::uint64_t late = 0;
  std::uint64_t rejected_in_a_row = 0;  // outliers since the last accepted measurement
  double last_delay = 0.0;  // [s] receive time - stamp
  Eigen::VectorXd last_innovation;
  double last_mahalanobis = 0.0;
};

class SensorAdapter
{
public:
  virtual ~SensorAdapter() = default;

  /// `name` is the sensor's key in the config; `params` reads "sensors.<name>.".
  virtual void initialize(
    const AdapterContext & context, const std::string & name, std::shared_ptr<const Params> params) = 0;

  /// State blocks this sensor adds, e.g. "<name>/gyro_bias". Called before the filter starts.
  virtual void addStates(StateLayoutBuilder & builder) const { (void)builder; }

  /// The layout is final: create the subscriptions.
  virtual void start(std::shared_ptr<const StateLayout> layout) = 0;

  /// The fuser's verdict on one of this sensor's measurements.
  virtual void onResult(const UpdateResult & result)
  {
    if (result.accepted) {
      ++statistics_.accepted;
      statistics_.rejected_in_a_row = 0;
    } else if (result.reason == "too late") {
      ++statistics_.late;
    } else {
      ++statistics_.rejected;
      ++statistics_.rejected_in_a_row;
    }
    statistics_.last_innovation = result.innovation;
    statistics_.last_mahalanobis = result.mahalanobis;
  }

  const SensorStatistics & statistics() const { return statistics_; }
  void clearRejectionsInARow() { statistics_.rejected_in_a_row = 0; }
  const std::string & name() const { return name_; }
  const std::string & type() const { return type_; }
  void setType(const std::string & type) { type_ = type; }

protected:
  std::string name_;
  std::string type_;
  SensorStatistics statistics_;
};

/// Base for adapters of one topic and one message type: subscription, common options, the
/// sensor's mounting from TF, statistics. Subclasses implement convert().
///
/// Common parameters (sensors.<name>.*):
///   topic                 required
///   frame                 overrides the message's frame_id (for the mounting)
///   use                   which quantities to fuse (adapter-specific names)
///   rejection_threshold   [sigma] Mahalanobis distance above which a measurement is rejected
///   max_rejections_in_a_row  after this many outliers in a row the filter widens its
///                         position and yaw uncertainty so it can take this sensor again
///                         (0: never; for absolute sensors like GNSS, so a lost estimate
///                         recovers instead of rejecting everything)
///   max_delay             [s] older measurements (receive time - stamp) are dropped
///   as_input              an input for the motion model instead of a measurement
///   enabled
/// Noise values are read at every message, so `ros2 param set` changes them live.
template<typename MessageT>
class TopicAdapter : public SensorAdapter
{
public:
  void initialize(
    const AdapterContext & context, const std::string & name, std::shared_ptr<const Params> params) override
  {
    context_ = context;
    name_ = name;
    params_ = std::move(params);
    topic_ = params_->getString("topic", "");
    if (topic_.empty()) {
      throw std::invalid_argument("sensors." + name + ".topic is missing");
    }
    frame_ = params_->getString("frame", "");
    as_input_ = params_->getBool("as_input", false);
    enabled_ = params_->getBool("enabled", true);
    use_ = params_->getStrings("use", defaultUse());
    configure(*params_);
  }

  void start(std::shared_ptr<const StateLayout> layout) override
  {
    layout_ = std::move(layout);
    if (!enabled_) {
      return;
    }
    subscription_ = context_.node->create_subscription<MessageT>(
      topic_, rclcpp::SensorDataQoS(),
      [this](typename MessageT::ConstSharedPtr message) { handle(*message); });
  }

protected:
  /// Build this message's measurements (or inputs) and send them with send().
  virtual void convert(const MessageT & message) = 0;
  /// Adapter-specific parameters, after the common ones are read.
  virtual void configure(const Params & params) { (void)params; }
  /// What `use` is when the config does not say.
  virtual std::vector<std::string> defaultUse() const { return {}; }

  bool uses(const std::string & quantity) const
  {
    return std::find(use_.begin(), use_.end(), quantity) != use_.end();
  }
  bool asInput() const { return as_input_; }
  const Params & params() const { return *params_; }
  const AdapterContext & context() const { return context_; }
  const StateLayout & layout() const { return *layout_; }

  /// The sensor frame in the body frame, from TF (cached: sensors do not move on the car).
  std::optional<Eigen::Isometry3d> mount(const std::string & message_frame)
  {
    const std::string frame = frame_.empty() ? message_frame : frame_;
    auto cached = mounts_.find(frame);
    if (cached != mounts_.end()) {
      return cached->second;
    }
    if (frame.empty() || frame == context_.base_frame) {
      mounts_[frame] = Eigen::Isometry3d::Identity();
      return Eigen::Isometry3d::Identity();
    }
    try {
      const auto t = context_.tf->lookupTransform(context_.base_frame, frame, tf2::TimePointZero);
      const Eigen::Isometry3d mount = tf2::transformToEigen(t);
      mounts_[frame] = mount;
      return mount;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        context_.node->get_logger(), *context_.node->get_clock(), 5000,
        "%s: no transform %s -> %s yet: %s", name_.c_str(), context_.base_frame.c_str(), frame.c_str(),
        error.what());
      return std::nullopt;
    }
  }

  /// The noise of a quantity: `sensors.<name>.<key>` (n variances, or n*n values) if set,
  /// else the message's covariance times `covariance_scale` if it is valid, else nothing.
  std::optional<Eigen::MatrixXd> noise(const std::string & key, const Eigen::MatrixXd & from_message) const
  {
    const int n = static_cast<int>(from_message.rows());
    const std::vector<double> values = params_->getDoubles(key, {});
    if (values.size() == static_cast<std::size_t>(n)) {
      return Eigen::VectorXd::Map(values.data(), n).asDiagonal().toDenseMatrix();
    }
    if (values.size() == static_cast<std::size_t>(n * n)) {
      return Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        values.data(), n, n);
    }
    if (!values.empty()) {
      RCLCPP_ERROR_THROTTLE(
        context_.node->get_logger(), *context_.node->get_clock(), 5000,
        "%s: %s needs %d or %d values", name_.c_str(), key.c_str(), n, n * n);
      return std::nullopt;
    }
    if (from_message.diagonal().minCoeff() > 0.0 && from_message.allFinite()) {
      return params_->getDouble("covariance_scale", 1.0) * from_message;
    }
    RCLCPP_WARN_THROTTLE(
      context_.node->get_logger(), *context_.node->get_clock(), 5000,
      "%s: the message has no covariance; set sensors.%s.%s", name_.c_str(), name_.c_str(), key.c_str());
    return std::nullopt;
  }

  Stamp stamp(const builtin_interfaces::msg::Time & time) const
  {
    return static_cast<Stamp>(time.sec) * 1000000000LL + time.nanosec;
  }

  /// Sends a measurement with the sensor's name and rejection threshold (the parameter
  /// `threshold_key` if set, else rejection_threshold).
  void send(Measurement measurement, const std::string & threshold_key = "")
  {
    measurement.source = name_;
    const double threshold = params_->getDouble("rejection_threshold", 5.0);
    measurement.rejection_threshold =
      threshold_key.empty() ? threshold : params_->getDouble(threshold_key, threshold);
    context_.sink->add(std::move(measurement));
  }
  void send(Input input)
  {
    input.source = name_;
    context_.sink->add(std::move(input));
  }

private:
  void handle(const MessageT & message)
  {
    ++statistics_.received;
    const double delay = toSeconds(context_.node->now().nanoseconds() - stamp(message.header.stamp));
    statistics_.last_delay = delay;
    if (delay > params_->getDouble("max_delay", 0.5)) {
      ++statistics_.late;
      return;
    }
    convert(message);
  }

  AdapterContext context_;
  std::shared_ptr<const Params> params_;
  std::string topic_;
  std::string frame_;
  bool as_input_ = false;
  bool enabled_ = true;
  std::vector<std::string> use_;
  std::shared_ptr<const StateLayout> layout_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  std::map<std::string, Eigen::Isometry3d> mounts_;
};

}  // namespace sac_localization

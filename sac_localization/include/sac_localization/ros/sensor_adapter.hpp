// Sensor adapter: turns a ROS message into measurements and inputs. A plugin (pluginlib,
// base class sac_localization::SensorAdapter); the standard ones are in
// sac_localization_adapters. One adapter instance per sensor in the config:
//
//   sensors:
//     names: [middle_imu, gnss_front_right]
//     middle_imu:
//       type: imu                     # plugin (lookup name in the adapters' plugins.xml)
//       topic: /sac/sensors/middle_imu/imu
//       ...                           # the adapter's parameters (see SensorOptions)
//
// Adapters know ROS and message types; they do not know the estimator. They read the
// sensor's mounting from TF and build measurements from the models in
// core/measurement_models.hpp.

#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
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
  double last_delay = 0.0;  // [s] receive time - stamp
};

class SensorAdapter
{
public:
  virtual ~SensorAdapter() = default;

  /// `name` is the sensor's key in the config; `params` reads "sensors.<name>.".
  virtual void initialize(const AdapterContext & context, const std::string & name, const Params & params) = 0;

  /// State blocks this sensor adds, e.g. "<name>/gyro_bias". Called before the filter starts.
  virtual void addStates(StateLayoutBuilder & builder) const { (void)builder; }

  /// The layout is final: create the subscriptions.
  virtual void start(std::shared_ptr<const StateLayout> layout) = 0;

  /// The fuser's verdict on one of this sensor's measurements.
  virtual void onResult(const UpdateResult & result) { (void)result; }

  virtual SensorStatistics statistics() const = 0;
  virtual const std::string & name() const = 0;
};

/// Options every topic-based adapter shares, read by TopicAdapter.
struct SensorOptions
{
  std::string topic;
  std::optional<std::string> frame;       // overrides the message's frame_id
  std::vector<std::string> use;           // which quantities to fuse, e.g. ["angular_velocity"]
  std::optional<std::vector<double>> covariance;  // overrides the message's (diagonal)
  double covariance_scale = 1.0;          // multiplies the message's covariance
  double rejection_threshold = 5.0;       // [sigma]
  double max_delay = 0.5;                 // [s] older measurements are dropped
  bool as_input = false;                  // an input for the motion model instead of a measurement
  bool enabled = true;
};

/// Base for adapters of one topic and one message type: subscription, common options, the
/// sensor's mounting from TF, statistics. Subclasses implement convert().
template<typename MessageT>
class TopicAdapter : public SensorAdapter
{
public:
  void initialize(const AdapterContext & context, const std::string & name, const Params & params) override;
  void start(std::shared_ptr<const StateLayout> layout) override;
  void onResult(const UpdateResult & result) override;
  SensorStatistics statistics() const override { return statistics_; }
  const std::string & name() const override { return name_; }

protected:
  /// Build this message's measurements (or inputs) and send them to context().sink.
  virtual void convert(const MessageT & message) = 0;
  /// Adapter-specific parameters, after the common ones are read.
  virtual void configure(const Params & params) { (void)params; }

  /// The sensor frame in the body frame, from TF (cached; the sensors do not move).
  std::optional<Eigen::Isometry3d> mount(const std::string & frame_id);
  /// The noise covariance to use: the config's, or the message's scaled.
  Eigen::MatrixXd noise(const Eigen::MatrixXd & message_covariance) const;
  bool uses(const std::string & quantity) const;
  Stamp stamp(const builtin_interfaces::msg::Time & time) const;

  const AdapterContext & context() const { return context_; }
  const SensorOptions & options() const { return options_; }
  const StateLayout & layout() const { return *layout_; }

private:
  AdapterContext context_;
  std::string name_;
  SensorOptions options_;
  std::shared_ptr<const StateLayout> layout_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
  std::map<std::string, Eigen::Isometry3d> mounts_;
  SensorStatistics statistics_;
};

}  // namespace sac_localization

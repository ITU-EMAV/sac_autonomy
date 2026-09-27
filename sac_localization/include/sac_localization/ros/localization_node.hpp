// The localization node: builds a filter from the config and runs it.
//
// From the YAML it loads (pluginlib) the estimator, the motion model and one adapter per
// sensor, builds the state layout, and then, at `frequency`:
//   fuser.update(now) -> outputs (TF, odometry, fix, status).
//
// The same node runs twice with two configs (REP 105):
//   local  (world_frame: odom): IMUs, wheels, constraints; smooth, never jumps; publishes
//          odom -> base_footprint
//   global (world_frame: map):  plus GNSS/heading/pose; publishes map -> odom so that
//          map -> odom -> base_footprint equals its estimate
//
// Noise parameters are ROS parameters: they can be changed while it runs (ros2 param set,
// Lichtblick's parameter panel) and take effect at the next update.

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sac_localization/core/estimator.hpp"
#include "sac_localization/core/fuser.hpp"
#include "sac_localization/core/geodesy.hpp"
#include "sac_localization/core/motion_model.hpp"
#include "sac_localization/ros/outputs.hpp"
#include "sac_localization/ros/sensor_adapter.hpp"

namespace sac_localization
{

class LocalizationNode : public rclcpp::Node, public MeasurementSink
{
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // MeasurementSink
  void add(Measurement measurement) override;
  void add(Input input) override;

private:
  void loadPlugins();
  void tick();
  /// The initial belief from the config (initial_state, initial_covariance), or from a pose.
  Belief initialBelief(const std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> & pose) const;
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);
  void onReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::string world_frame_;
  std::string base_frame_;
  std::optional<MapFrame> map_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  pluginlib::ClassLoader<Estimator> estimator_loader_;
  pluginlib::ClassLoader<MotionModel> motion_model_loader_;
  pluginlib::ClassLoader<SensorAdapter> adapter_loader_;
  std::vector<std::shared_ptr<SensorAdapter>> adapters_;

  std::shared_ptr<const StateLayout> layout_;
  std::unique_ptr<Fuser> fuser_;
  std::vector<std::unique_ptr<Output>> outputs_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sac_localization

// The localization node: builds a filter from the config and runs it.
//
// From the YAML it loads (pluginlib) the estimator, the motion model and one adapter per
// sensor, builds the state layout, and then, at `frequency`:
//   fuser.update(now) -> outputs (TF, odometry, fix, status, error).
//
// The same node runs twice with two configs (REP 105):
//   local  (world_frame: odom): IMUs, wheels, constraints; smooth, never jumps; publishes
//          odom -> base_footprint
//   global (world_frame: map):  plus GNSS; publishes map -> odom so that
//          map -> odom -> base_footprint equals its estimate
//
// Start (initial_state.pose_from):
//   origin        identity pose (the local filter)
//   config        initial_state.position [x, y, z] and initial_state.yaw
//   gnss          averages the gnss_position sensors for initial_state.gnss_duration seconds:
//                 position from the antennas, yaw from the baseline between two antennas
//   initial_pose  waits for a pose on initial_pose_topic
// A pose on initial_pose_topic (if set) restarts the filter at any time; ~/reset restarts
// it from pose_from. When the clock jumps back (the simulation restarted) it restarts too.
//
// Recovery: a sensor with sensors.<name>.max_rejections_in_a_row (e.g. GNSS) that is rejected
// that many times in a row widens the position and yaw uncertainty (recovery_covariance.*),
// so a lost estimate comes back instead of rejecting the sensor for ever.
//
// Noise parameters are ROS parameters: adapters read theirs at every message, the motion
// model is re-initialized when a motion_model.* parameter changes.

#pragma once

#include <map>
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
#include "sac_localization/ros/ros_params.hpp"
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
  struct GnssAverage
  {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();
    int count = 0;
  };

  void tick();
  /// The belief to start from, once the start condition is met.
  std::optional<Belief> initialBelief(Stamp now);
  Belief beliefAt(Stamp stamp, const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation, double yaw_variance) const;
  void restart();
  /// Widens the uncertainty when a sensor with max_rejections_in_a_row keeps disagreeing.
  void recoverIfLost();
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  RosParams root_;
  std::string world_frame_;
  std::string base_frame_;
  std::optional<MapFrame> map_;
  std::string estimator_type_;
  std::string motion_model_type_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  pluginlib::ClassLoader<Estimator> estimator_loader_;
  pluginlib::ClassLoader<MotionModel> motion_model_loader_;
  pluginlib::ClassLoader<SensorAdapter> adapter_loader_;
  std::shared_ptr<Estimator> estimator_;
  std::shared_ptr<MotionModel> motion_model_;
  std::shared_ptr<RosParams> motion_model_params_;
  std::vector<std::shared_ptr<SensorAdapter>> adapters_;
  std::map<std::string, std::shared_ptr<SensorAdapter>> adapters_by_name_;

  std::shared_ptr<const StateLayout> layout_;
  std::unique_ptr<Fuser> fuser_;
  std::vector<std::unique_ptr<Output>> outputs_;

  // Start
  std::string pose_from_;
  std::map<std::string, GnssAverage> gnss_;
  std::optional<Stamp> gnss_since_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> initial_pose_;
  bool reinitialize_motion_model_ = false;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameters_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sac_localization

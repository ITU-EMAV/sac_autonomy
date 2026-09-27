// What the node publishes after each update. Each output is switched on and named in the
// config under "outputs.":
//   TfOutput        world -> base (local filter), or map -> odom (global filter)
//   OdometryOutput  nav_msgs/Odometry: pose, twist and covariance in world_frame
//   FixOutput       sensor_msgs/NavSatFix: the estimate as latitude/longitude (needs a datum)
//   StatusOutput    sac_localization_msgs/FilterStatus: per sensor accepted/rejected,
//                   innovation, Mahalanobis distance, delay (for tuning)
//   ErrorOutput     simulation only: position and yaw error against a ground truth topic

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sac_localization/core/fuser.hpp"
#include "sac_localization/core/params.hpp"
#include "sac_localization/ros/sensor_adapter.hpp"

namespace sac_localization
{

/// What an output gets after each update.
struct OutputContext
{
  const Belief & belief;                     // at `now`
  const std::vector<UpdateResult> & results;  // of this update
  const std::vector<std::shared_ptr<SensorAdapter>> & adapters;
  rclcpp::Time now;
};

class Output
{
public:
  virtual ~Output() = default;
  virtual void initialize(rclcpp::Node & node, const AdapterContext & context, const Params & params) = 0;
  virtual void publish(const OutputContext & output) = 0;
};

/// The outputs switched on in the config.
std::vector<std::unique_ptr<Output>> makeOutputs(
  rclcpp::Node & node, const AdapterContext & context, const Params & params);

}  // namespace sac_localization

// What the node publishes after each update, set under "outputs." in the config:
//   tf: true             world -> base (local filter, world_frame == odom_frame), or
//                        map -> odom (global filter) so that map -> odom -> base is the estimate
//   local_odometry: <topic>
//                        global filter: the local filter's odometry, to compute map -> odom
//                        with odom -> base at exactly the same time (else from TF at that time)
//   odometry: <topic>    nav_msgs/Odometry: pose, twist and covariance in world_frame
//   fix: <topic>         sensor_msgs/NavSatFix: the estimate as latitude/longitude (needs a datum)
//   status: <topic>      sac_localization_msgs/FilterStatus: per sensor accepted/rejected,
//                        innovation, Mahalanobis distance, delay (for tuning)
//   ground_truth: <topic>, error: <topic>
//                        simulation only: sac_localization_msgs/EstimateError against a
//                        geometry_msgs/PoseStamped ground truth in world_frame
// An empty topic switches an output off.

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
  const Belief & belief;                      // at `now`
  const std::vector<UpdateResult> & results;  // of this update
  const std::vector<std::shared_ptr<SensorAdapter>> & adapters;
  std::string estimator;
  std::string motion_model;
};

class Output
{
public:
  virtual ~Output() = default;
  virtual void publish(const OutputContext & output) = 0;
};

/// The outputs switched on in the config (`params` is the node's root: reads "outputs.*" and
/// "odom_frame").
std::vector<std::unique_ptr<Output>> makeOutputs(
  rclcpp::Node & node, const AdapterContext & context, const Params & params);

}  // namespace sac_localization

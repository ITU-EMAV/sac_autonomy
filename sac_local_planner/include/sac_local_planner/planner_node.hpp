// The local planner node: follows the global route (/sac/planning/path) around the
// perception grid's obstacles (/sac/perception/grid) and publishes a short trajectory with
// speeds (/sac/planning/trajectory) for the controller. A component
// (sac_local_planner::LocalPlannerNode), so it can share a process with the perception.
//
// Parameters (config/sim.yaml):
//   map_frame (map), base_frame (base_footprint), rate [Hz] (10)
//   occupied_threshold (65): grid cells from this probability [%] are obstacles
//   max_grid_age [s] (1.0): with an older grid (or none) the car stops: no blind driving
//   generator.type (frenet_lattice) and its parameters under generator.
//   costs.names: [...]; costs.<name>.type (default: the name), .weight, and its parameters
//   footprint.offsets, .radius, .safety_margin; limits.max_speed, .max_lateral_acceleration,
//   .max_acceleration, .max_deceleration, .stop_margin (local_planner.hpp)
//   moving.enabled, .time_margin, .horizon, .safety, .speed_uncertainty, .max_uncertainty
//   (local_planner.hpp); moving.max_age [s] (0.5): older objects are not used;
//   moving.small_size [m] (1.2), moving.min_speed [m/s] (0.5): objects not (yet) called moving
//   but this small and surely this fast are taken for yielding (local_planner.hpp);
//   moving.hold [s] (1.5): an object seen moving is kept that long after (going on at its
//   velocity), though its track drops out or stops being called moving for a moment
// Topics: path, grid, objects (sac_perception_msgs/TrackedObjects: the moving ones) in; ~/trajectory (sac_planning_msgs/Trajectory), ~/candidates
// (visualization_msgs/MarkerArray: free green, blocked red, the chosen one blue), ~/timing
// (diagnostic_msgs/DiagnosticArray) out.

#pragma once

#include <memory>
#include <map>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sac_perception_msgs/msg/tracked_objects.hpp>
#include <sac_planning_msgs/msg/trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "sac_local_planner/local_planner.hpp"
#include "sac_perception/ros_params.hpp"

namespace sac_local_planner
{

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options);

private:
  void onPath(const nav_msgs::msg::Path::ConstSharedPtr & message);
  void onGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & message);
  void tick();
  void publish(const PlanResult & result, const PlanningContext & context, const rclcpp::Time & stamp);
  void publishStop(const PlanningContext & context, const rclcpp::Time & stamp);

  sac_perception::RosParams root_;
  std::string map_frame_;
  std::string base_frame_;
  int occupied_threshold_ = 65;
  double max_grid_age_ = 1.0;

  pluginlib::ClassLoader<TrajectoryGenerator> generator_loader_;
  pluginlib::ClassLoader<CostFunction> cost_loader_;
  std::unique_ptr<LocalPlanner> planner_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_subscription_;
  rclcpp::Subscription<sac_perception_msgs::msg::TrackedObjects>::SharedPtr objects_subscription_;
  sac_perception_msgs::msg::TrackedObjects::ConstSharedPtr objects_;
  double objects_max_age_ = 0.5;
  double objects_hold_ = 1.5;
  ObjectSelection selection_;
  struct Held
  {
    MovingObject object;   // in the map, at `seen`
    rclcpp::Time seen;
  };
  std::map<uint32_t, Held> held_;   // by the perception's id
  std::vector<MovingObject> moving_;
  rclcpp::Publisher<sac_planning_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidates_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr timing_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  ReferencePath route_;
  double route_hint_ = -1.0;
  DistanceMap obstacles_;
  std::string grid_frame_;
  std::optional<rclcpp::Time> grid_stamp_;
  double edt_ms_ = 0.0;

  // The car's speed, from its pose over time
  std::optional<rclcpp::Time> last_pose_stamp_;
  Eigen::Vector2d last_position_ = Eigen::Vector2d::Zero();
  double speed_ = 0.0;

  double previous_target_d_ = 0.0;
  bool has_previous_ = false;
  double plan_ms_max_ = 0.0;
};

}  // namespace sac_local_planner

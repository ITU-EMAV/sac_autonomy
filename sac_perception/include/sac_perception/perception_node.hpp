// The perception node: the local occupancy grid from any number of sensors (sources.hpp),
// set up from YAML (config/sim.yaml). A component (sac_perception::PerceptionNode), so the
// planner can run in the same process.
//
// Parameters:
//   grid_frame (odom), base_frame (base_footprint)
//   grid.size [m] (80), grid.resolution [m] (0.2), rate [Hz] (20): publishing and moving
//   grid.recenter_distance [m] (2): the window moves once the car is this far off its centre
//   occupied_threshold (0.65): the probability from which a cell counts as occupied
//   sources.names: [...] and each source's parameters under sources.<name>.
// Publishes:
//   ~/grid     nav_msgs/OccupancyGrid in grid_frame, all layers combined (remapped to
//              /sac/perception/grid)
//   ~/timing   diagnostic_msgs/DiagnosticArray: per source the processing time [ms] (last,
//              mean, max) and skipped messages, and the grid step's time

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sac_perception/filters.hpp"
#include "sac_perception/ros_params.hpp"
#include "sac_perception/sources.hpp"

namespace sac_perception
{

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options);

private:
  void tick();

  RosParams root_;
  std::string grid_frame_;
  std::string base_frame_;
  float occupied_threshold_ = 0.65f;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  SharedGrid grid_;
  pluginlib::ClassLoader<PointFilter> filter_loader_;
  pluginlib::ClassLoader<GridSource> source_loader_;
  std::vector<std::shared_ptr<GridSource>> sources_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr timing_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_;
  bool ticked_ = false;
  double grid_ms_max_ = 0.0;
};

}  // namespace sac_perception

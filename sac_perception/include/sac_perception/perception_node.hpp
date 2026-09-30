// The perception node: the local occupancy grid from any number of sensors (sources.hpp),
// set up from YAML (config/sim.yaml). A component (sac_perception::PerceptionNode), so the
// planner can run in the same process.
//
// Parameters:
//   grid_frame (odom), base_frame (base_footprint)
//   grid.size [m] (80), grid.resolution [m] (0.2), rate [Hz] (20): publishing and moving
//   grid.recenter_distance [m] (2): the window moves once the car is this far off its centre
//   map.type (direct_projection): how the sensors' data is kept and projected to the grid
//     (map_representation.hpp), its parameters under map.
//   vehicle.from: the car's box, for filters that follow the car (crop_box from: vehicle,
//     height_band max_from: vehicle)
//       robot_description  its URDF on vehicle.topic (/robot_description, from
//                          robot_state_publisher); the sources start once it has come
//       box                vehicle.box: [min x, y, z, max x, y, z] in base_frame
//       none               (default) unknown
//   sources.names: [...] and each source's parameters under sources.<name>.
//   publish_map (false), publish_map_rate [Hz] (5): what the map holds on ~/map
// Publishes:
//   ~/grid     nav_msgs/OccupancyGrid in grid_frame, all layers combined (remapped to
//              /sac/perception/grid)
//   ~/map      sensor_msgs/PointCloud2 in grid_frame (with publish_map): the map's elements
//              (voxels, or occupied cells at their ground), fields x, y, z, occupancy [%],
//              blocks (1: in the grid, 0: kept but not in the way, e.g. over the car),
//              source (index in sources.names)
//   ~/timing   diagnostic_msgs/DiagnosticArray: per source the processing time [ms] (last,
//              mean, max) and skipped messages, and the grid step's time with the map's
//              numbers

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
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
  /// The map and the sources, once the car's box is known
  void start(const VehicleBox & vehicle);
  void tick();
  void publishMap(const rclcpp::Time & stamp);

  RosParams root_;
  std::string grid_frame_;
  std::string base_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  // The loaders before what they made: members go in reverse order
  pluginlib::ClassLoader<MapRepresentation> map_loader_;
  pluginlib::ClassLoader<PointFilter> filter_loader_;
  SharedMap map_;
  VehicleBox vehicle_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_subscription_;
  pluginlib::ClassLoader<GridSource> source_loader_;
  std::vector<std::shared_ptr<GridSource>> sources_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr timing_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  double map_period_ = 0.2;  // [s]
  rclcpp::Time last_map_;
  std::vector<MapPoint> map_points_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_;
  bool ticked_ = false;
  double grid_ms_max_ = 0.0;
};

}  // namespace sac_perception

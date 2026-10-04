// TODO: Remove unnecessary variables.
#ifndef PLANNER__OCCUPANCY_GRID_HPP_
#define PLANNER__OCCUPANCY_GRID_HPP_

// ============================================================================
// SYSTEM INCLUDES
// ============================================================================
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// ============================================================================
// ROS2 INCLUDES
// ============================================================================
#include <rclcpp/rclcpp.hpp>

// ROS2 Messages
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>

// Custom Messages
#include <cluster_msgs/msg/cluster_points_array.hpp>
#include <sac_interfaces/msg/path_ids.hpp>

// TF2 Libraries
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ============================================================================
// THIRD-PARTY INCLUDES
// ============================================================================
// Lanelet2
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

// OpenCV
#include <opencv2/opencv.hpp>

// Clipper
#include "clipper2/clipper.h"

// ============================================================================
// DEBUG INCLUDES (TODO: Remove after debugging)
// ============================================================================
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace smart_car {

class OccupancyGrid : public rclcpp::Node
{
public:
    // ========================================================================
    // CONSTRUCTOR
    // ========================================================================
    OccupancyGrid(const rclcpp::NodeOptions & options);

private:
    // ========================================================================
    // GRID CONFIGURATION
    // ========================================================================
    double grid_resolution_;    // Resolution of the occupancy grid in meters per cell
    int    grid_size_;         // Number of cells in one dimension of the grid
    bool   occupancy_ready;    // Flag to check if the occupancy grid is ready
    bool   debug_mode;         // Flag to enable debug mode

    // ========================================================================
    // TRANSFORM HANDLING
    // ========================================================================
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ========================================================================
    // MAP AND VEHICLE CONFIGURATION
    // ========================================================================
    std::string osm_path;      // Path to the OSM file
    double origin_x;           // Grid origin X coordinate
    double origin_y;           // Grid origin Y coordinate
    
    double vehicle_width;      // Vehicle width in meters
    double vehicle_length;     // Vehicle length in meters
    double safety_margin;      // Safety margin in meters

    lanelet::LaneletMapPtr lanelet_map_;  // Pointer to the lanelet map

    // ========================================================================
    // CURRENT STATE
    // ========================================================================
    geometry_msgs::msg::Pose pose;       // Vehicle's current pose
    sac_interfaces::msg::PathIds path_;     // Path IDs for the vehicle to follow

    // ========================================================================
    // ROS2 PUBLISHERS
    // ========================================================================
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lanelet_marker_pub_;  // Debug

    // ========================================================================
    // ROS2 SUBSCRIBERS
    // ========================================================================
    rclcpp::Subscription<cluster_msgs::msg::ClusterPointsArray>::SharedPtr cluster_sub_;
    rclcpp::Subscription<sac_interfaces::msg::PathIds>::SharedPtr path_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_;

    // ========================================================================
    // HELPER FUNCTIONS
    // ========================================================================
    void readLaneletMap();
    std::optional<cv::Point> transform_point(const lanelet::ConstPoint3d& pt) const;

    // ========================================================================
    // ROS2 CALLBACK FUNCTIONS
    // ========================================================================
    void localization_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
    void clustersCallback(const cluster_msgs::msg::ClusterPointsArray & msg);
    void pathCallback(const sac_interfaces::msg::PathIds & msg);
};

} // namespace smart_car

#endif  // PLANNER__OCCUPANCY_GRID_HPP_
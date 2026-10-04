#ifndef PLANNER__TRAJECTORY_PLANNER_HPP_
#define PLANNER__TRAJECTORY_PLANNER_HPP_

// Standard library includes
#include <string>
#include <chrono>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

// Lanelet2 includes
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

// ROS2 message includes
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Custom message includes
#include <sac_interfaces/msg/path_ids.hpp>
#include <sac_interfaces/msg/pipeline_timing.hpp>

// TF2 includes
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

// Local includes
#include "AStar.hpp"
#include "lattice_generator.hpp"
#include "spline.h"

namespace smart_car
{
    // Which search runs once an in-lane obstacle takes the vehicle off the
    // centerline. Selected from the config through the `planner_type` parameter.
    enum class PlannerType
    {
        AStar,
        Lattice
    };

    class TrajectoryPlanner : public rclcpp::Node
    {
    public:
        TrajectoryPlanner(const rclcpp::NodeOptions & options);

    private:
        // ========== Configuration Parameters ==========
        std::string osm_path;
        bool follow_centerline;
        double curvature_contributing_factor;
        double centerline_contributing_factor;
        int clearance;
        double unknown_cell_cost;
        double occupancy_cost_factor;
        int lethal_cost_threshold;
        double turn_penalty;
        double planning_time_budget;
        int ensuring_factor;

        // ========== Lattice Planner Configuration ==========
        PlannerType planner_type;
        double lattice_planner_horizon;
        double lattice_behind_distance;
        int lattice_window_behind;
        int lattice_window_ahead;
        bool lattice_publish_debug;

        // ========== Map and Coordinate Data ==========
        lanelet::LaneletMapPtr lanelet_map;
        double origin_x;
        double origin_y;

        // ========== Current State Data ==========
        geometry_msgs::msg::Point current_position;
        sac_interfaces::msg::PathIds::SharedPtr path;
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        bool switch_controller_received = false;
        double current_speed = 0.0;

        // ========== Planners ==========
        AStar::Generator astar;
        Lattice::Generator lattice;

        // ========== TF2 Components ==========
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        // ========== ROS2 Subscribers ==========
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub;
        rclcpp::Subscription<sac_interfaces::msg::PathIds>::SharedPtr path_sub;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_controller_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;

        // ========== ROS2 Publishers ==========
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        rclcpp::Publisher<sac_interfaces::msg::PipelineTiming>::SharedPtr timing_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lattice_debug_pub;
        std::chrono::steady_clock::time_point plan_start_;
        rclcpp::Time pose_stamp_{0, 0, RCL_ROS_TIME};
        uint64_t plan_sequence_{0};
        void publishTrajectory(const nav_msgs::msg::Path & trajectory);

        // ========== Helper Functions ==========
        void readLaneletMap();
        void followCenterline(const geometry_msgs::msg::PoseWithCovarianceStamped & msg);
        std::vector<double> movingAverage(const std::vector<double>& data, int windowSize);

        // Avoidance planners. Both take the route centerline collected by
        // poseCallback and publish onto the trajectory topic.
        void planWithAStar(
            const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
            const std::vector<geometry_msgs::msg::PointStamped> & centerline_points);
        void planWithLattice(
            const geometry_msgs::msg::PoseWithCovarianceStamped & msg,
            const std::vector<geometry_msgs::msg::PointStamped> & centerline_points);
        void publishLatticeDebug(
            const std::string & frame_id, double origin_x, double origin_y,
            double cos_yaw, double sin_yaw);

        // ========== ROS2 Callback Functions ==========
        void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);
        void pathCallback(const sac_interfaces::msg::PathIds & path_ids);
        void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid grid);
        void switchControllerCallback(const std_msgs::msg::Bool & msg);
        void odometryCallback(const nav_msgs::msg::Odometry & msg);
    };
} // namespace smart_car

#endif  // PLANNER__TRAJECTORY_PLANNER_HPP_

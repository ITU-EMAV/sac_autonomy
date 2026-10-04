#ifndef PLANNER__GLOBAL_PLANNER_HPP_
#define PLANNER__GLOBAL_PLANNER_HPP_

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

// TF2 includes
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

// Lanelet2 includes
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// Custom message includes
#include <cluster_msgs/msg/cluster_points_array.hpp>
#include <cluster_msgs/msg/cluster_points.hpp>
#include <sac_interfaces/msg/camera_detection_array.hpp>

#include <visualization_msgs/msg/marker.hpp>
// Standard library includes
#include <vector>
#include <string>

namespace smart_car
{

    class GlobalPlanner : public rclcpp::Node
    {
    public:
        explicit GlobalPlanner(const rclcpp::NodeOptions &options);

    private:
        // ====== CORE FUNCTIONS ======
        void readLaneletMap();
        void readIds();

        // ====== ROS CALLBACK FUNCTIONS ======
        void rviz_goal_pose_callback(const geometry_msgs::msg::PoseStamped &msg);
        void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
        void cluster_callback(const cluster_msgs::msg::ClusterPointsArray &msg);
        void no_entry_arr_sub(const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg);
        void yoloCallback(const sac_interfaces::msg::CameraDetectionArray &msg);

        // ====== PLANNER FUNCTIONS ======
        void select_and_publish_nearest_station();
        void on_wait_timer();
        void remove_no_entry_stations();
        void goto_park();

        // ====== LANELET MAP VARIABLES ======
        lanelet::LaneletMapPtr lanelet_map;
        std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph;
        double origin_x;
        double origin_y;
        std::string osm_path;

        // ====== CONFIGURATION VARIABLES ======
        std::string station_name;
        std::string park_name;
        bool rviz_pose;

        // ====== STATE VARIABLES ======
        bool waiting_ = false;
        bool goal_pose_update = true;
        bool do_park = false;
        bool is_set = false;
        bool parked = false;
        bool is_finished = false;
        bool is_first_day = false;
        bool park_go = false;
        bool at_goal_station = false;
        double stopping_at_station_tolerance;
        double stopping_at_park_tolerance;

        // ====== TIMING VARIABLES ======
        rclcpp::Time arrival_time_;
        rclcpp::TimerBase::SharedPtr wait_timer_;

        // ====== TRANSFORM VARIABLES ======
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // ====== POSITION VARIABLES ======
        geometry_msgs::msg::Point current_position;
        geometry_msgs::msg::PoseStamped station1;
        geometry_msgs::msg::PoseStamped station2;
        geometry_msgs::msg::PoseStamped station3;
        geometry_msgs::msg::PoseStamped station4;
        std::vector<geometry_msgs::msg::PoseStamped> station_positions;

        // ====== ID COLLECTIONS ======
        std::vector<size_t> station_ids;
        std::vector<size_t> park_ids;
        std::vector<size_t> remaining_station_ids;
        std::vector<int32_t> no_entry_ids;
        std::vector<int32_t> no_park_lanes;
        std::vector<int> closed_park_ids;
        std::vector<int> PARK_LANES = {2605, 2612, 2619, 2626, 2633, 2640, 2647, 2654};

        // ====== CURRENT TARGETS ======
        lanelet::Id prev_lane_id;
        lanelet::Id parking_lot_entry_id;
        lanelet::Id first_goal_id;
        lanelet::Id station_goal_lanelet_id;
        size_t current_goal_station_id;

        // ========================
        // TIME & CLOCK VARIABLES
        // ========================
        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Time start_time_;
        rclcpp::Time thinking_free_park_time_;

        // ====== PARKING DATA ======
        std::vector<std::vector<std::pair<int, double>>> parking_data;

        // ====== ROS SUBSCRIBERS ======
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
        rclcpp::Subscription<cluster_msgs::msg::ClusterPointsArray>::SharedPtr cluster_sub;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr no_entry_sub;
        rclcpp::Subscription<sac_interfaces::msg::CameraDetectionArray>::SharedPtr yolo_sub;

        // ====== ROS PUBLISHERS ======
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr goal_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_limit_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr switch_to_centerline_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr station_marker_pub;

        // ====== VEHICLE STATE ENUM ====== // todo use this instead static bools
        enum VehicleState
        {
            DRIVING,
            STOPPED_AT_STATION,
            STOPPED_AT_PARKING,
            PARKING_DONE
        };
        VehicleState state = DRIVING; // Initial state
    };

} // namespace smart_car

#endif // PLANNER__GLOBAL_PLANNER_HPP_

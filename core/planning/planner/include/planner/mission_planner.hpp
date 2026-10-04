#ifndef PLANNER__MISSION_PLANNER_HPP_
#define PLANNER__MISSION_PLANNER_HPP_

// Standard Library Includes
#include <string>
#include <optional>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <array>
#include <memory>

// ROS2 Includes
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

// ROS2 Message Includes
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// Custom Message Includes
#include <sac_interfaces/msg/path_ids.hpp>
#include <sac_interfaces/msg/camera_detection_array.hpp>
#include <sac_interfaces/msg/telemetry.hpp>
#include <cluster_msgs/msg/cluster_points_array.hpp>

// Third-party Library Includes
#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point.hpp>

// Lanelet2 Includes
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// Type Aliases
using BoostPoint = boost::geometry::model::d2::point_xy<double>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

namespace smart_car
{
    class MissionPlanner : public rclcpp::Node
    {
    public:
        // Constructor
        MissionPlanner(const rclcpp::NodeOptions &options);

    private:
        // ========================
        // CONSTANTS
        // ========================
        static constexpr std::array<int, 9> PARK_LANES = {1101, 1103, 1105, 1107, 1109, 1111, 1113, 1115, 1117};

        // ========================
        // CORE FUNCTIONS
        // ========================
        void readLaneletMap();
        void load_the_graph();
        void get_station_ids();
        void switchPlanningMode();

        // ========================
        // UTILITY FUNCTIONS
        // ========================
        bool pointInPolygon(double x, double y, const std::vector<std::pair<double, double>> &poly);
        bool checkAtLeastOneRouteExists();
        double getLaneletYaw(const lanelet::ConstLanelet &lanelet);
        void checkPedestrianOnPath();
        void publish_no_entry_ids(int data, bool from_yolo);

        // ========================
        // ROS2 CALLBACK FUNCTIONS
        // ========================
        void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);
        void goalPoseCallback(const std_msgs::msg::Int32 &goal_pose_id);
        void yoloCallback(const sac_interfaces::msg::CameraDetectionArray &yolo_detections);
        void telemetryCallback(const sac_interfaces::msg::Telemetry &msg);
        void cluster_callback(const cluster_msgs::msg::ClusterPointsArray &msg);
        void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid &msg);

        // ========================
        // LANELET2 VARIABLES
        // ========================
        lanelet::LaneletMapPtr lanelet_map;
        lanelet::routing::RoutingGraphPtr routing_graph;
        lanelet::traffic_rules::TrafficRulesPtr traffic_rules = 
            lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
        lanelet::Lanelet current_lanelet;
        lanelet::Lanelet previous_lanelet;
        lanelet::Lanelet parking_entry_lanelet;
        lanelet::routing::LaneletPath shortest_path;
        std::vector<lanelet::BasicPoint2d> centerline_points;

        // ========================
        // COORDINATE & POSITION VARIABLES
        // ========================
        double origin_x;
        double origin_y;
        geometry_msgs::msg::Pose pose;
        geometry_msgs::msg::Point current_position;

        // ========================
        // PATH & NAVIGATION VARIABLES
        // ========================
        std::string osm_path;
        size_t goal_pose_id = 0;
        int32_t prev_goal_pose_id = 0;
        lanelet::Id current_goal_station_id = 0;
        lanelet::Id parking_lot_entry_id = 0;
        lanelet::Id previous_lane_id;
        size_t previous_centerpoint_idx = 0;
        sac_interfaces::msg::PathIds prev_path_ids;
        sac_interfaces::msg::PathIds path_ids;

        // ========================
        // STATE & FLAG VARIABLES
        // ========================
        bool force_replan = true;
        bool first_time = false;
        bool has_path = false;
        bool is_pose_start = false;
        bool a_star_running = false;
        bool instant_on_enable_ = true;
        bool switch_state_ = false;
        bool park_publish_flag_ = true;
        bool park_published_ = false;

        // ========================
        // STATION & ID VARIABLES
        // ========================
        std::vector<lanelet::Id> station_ids;
        std::vector<lanelet::Id> station_entry_ids;
        std::vector<lanelet::Id> remaining_station_ids;
        std::string STATION_NAME;
        std::vector<int32_t> no_entry_ids;
        std::vector<int32_t> prev_no_entry_ids;
        int park_id_ = 0;

        // ========================
        // DETECTION & PERCEPTION VARIABLES
        // ========================
        std::unordered_map<int, std::string> last_valid_detection_;
        cluster_msgs::msg::ClusterPointsArray bottom_points_array;
        rclcpp::Time last_yolo_msg_time;

        // ========================
        // VEHICLE & TELEMETRY VARIABLES
        // ========================
        std::shared_ptr<double> vehicle_speed_ptr;
        rclcpp::Time vehicle_speed_update_time;

        // ========================
        // ALGORITHM PARAMETERS
        // ========================
        double max_consider_dist_ = 25.0;
        double rear_ignore_dist_ = 2.0;
        double front_fov_rad_ = 120.0 * M_PI / 180.0;

        // ========================
        // TIME & CLOCK VARIABLES
        // ========================
        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Time start_time_;

        // ========================
        // TF2 VARIABLES
        // ========================
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // ========================
        // ROS2 SUBSCRIBERS
        // ========================
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goal_sub;
        rclcpp::Subscription<sac_interfaces::msg::CameraDetectionArray>::SharedPtr yolo_sub;
        rclcpp::Subscription<sac_interfaces::msg::Telemetry>::SharedPtr telemetry_sub;
        rclcpp::Subscription<cluster_msgs::msg::ClusterPointsArray>::SharedPtr cluster_sub;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub;

        // ========================
        // ROS2 PUBLISHERS
        // ========================
        rclcpp::Publisher<sac_interfaces::msg::PathIds>::SharedPtr path_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stop_distance_pub;
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr NoEntryPub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr switch_controller_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr park_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr traffic_light_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr behavioral_pub_;
    };
} // namespace smart_car

#endif // PLANNER__MISSION_PLANNER_HPP_
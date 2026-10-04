#ifndef PLANNER__CONTROLLER_HPP_
#define PLANNER__CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sac_interfaces/msg/actuator_command.hpp>
#include <sac_interfaces/msg/pipeline_timing.hpp>
#include <sac_interfaces/msg/speed_constraint.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace smart_car
{
    /**
     * Pure-pursuit path tracking controller.
     *
     * C++ port of sac_trajectory_tracking/controller_global.py.
     *
     *   * Uses a geometric lookahead point instead of a fixed path index.
     *   * Projects onto the nearest segment instead of the nearest node.
     *   * Interpolates the lookahead point on the final segment.
     *   * Wraps alpha to [-pi, pi].
     *   * Publishes the steering angle in cmd_vel.angular.z [rad].
     *   * Clamps curvature using the Ackermann steering limit.
     *   * Checks goal completion using remaining arc length.
     *   * Publishes the lookahead visualization as a MarkerArray.
     *   * Limits speed from path curvature: v <= sqrt(a_lat_max / |kappa|).
     *   * Curvature lookahead gain defaults to zero.
     */
    class Controller : public rclcpp::Node
    {
    public:
        explicit Controller(const rclcpp::NodeOptions &options);
        ~Controller() override;

    private:
        using Vec2 = std::array<double, 2>;

        /// Path projection, segment index, and remaining arc length.
        struct Projection
        {
            Vec2 start{{0.0, 0.0}};
            std::size_t i0{0};
            double s_remain{0.0};
        };

        /// Lookahead target, path index, and remaining arc length.
        struct Lookahead
        {
            Vec2 target{{0.0, 0.0}};
            std::size_t idx{0};
            double s_remain{0.0};
        };

        // ------------------------------------------------------------------ //
        // Callbacks
        // ------------------------------------------------------------------ //
        void pathCallback(const nav_msgs::msg::Path &msg);
        void locCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
        void controllerCallback();
        void controlLoop();

        // ------------------------------------------------------------------ //
        // Path curvature and projection
        // ------------------------------------------------------------------ //
        Projection projectOnPath(const Vec2 &curr) const;
        double estimatePathCurvature(std::size_t i0, const Vec2 &start, double window_m) const;
        std::optional<Lookahead> findLookaheadPoint(double Ld, const Projection &proj) const;

        /// Publishes a full-brake CONTROLLED_STOP command and reports why on /control/controller_state.
        void stop(const std::string &reason, const std::string &detail = "");
        /// Publishes and logs the controller state only when it changes; detail is informational.
        void reportState(const std::string &state, const std::string &detail = "");
        void publishLookaheadMarker(const Vec2 &target, const Vec2 &curr, double Ld,
                                    bool reached = false);
        void publishTiming(double jitter_ms, double execution_ms, double source_age_ms,
                           double snapshot_lock_ms, double path_projection_ms,
                           double control_compute_ms, double publish_ms);

        // ------------------------------------------------------------------ //
        // ROS interface
        // ------------------------------------------------------------------ //
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<sac_interfaces::msg::ActuatorCommand>::SharedPtr command_publisher_;
        /// Direct Twist output. It bypasses the guardian; an empty cmd_vel_topic disables it.
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::Publisher<sac_interfaces::msg::PipelineTiming>::SharedPtr timing_publisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
        std::string last_state_;
        rclcpp::CallbackGroup::SharedPtr callback_group_;

        // ------------------------------------------------------------------ //
        // Parameters
        // ------------------------------------------------------------------ //
        struct SpeedInput {
            rclcpp::Subscription<sac_interfaces::msg::SpeedConstraint>::SharedPtr subscription;
            std::optional<rclcpp::Time> stamp;
            std::chrono::steady_clock::time_point receipt;
            double speed{0.0};
            std::string topic;
            std::string reason;
            /// Why the last received message was rejected; empty after an accepted message.
            std::string rejected;
        };
        std::vector<std::shared_ptr<SpeedInput>> speed_inputs_;
        double constraint_timeout_{0.3};
        bool enforce_freshness_{true};
        double future_stamp_tolerance_{0.1};
        double command_acceleration_{0.5};
        double command_deceleration_{1.5};
        double previous_target_speed_{0.0};
        std::chrono::steady_clock::time_point previous_target_time_{std::chrono::steady_clock::now()};
        double linear_velocity_;   // m/s
        double min_linear_velocity_;  // m/s, used for a valid reactive path
        double full_speed_path_length_;  // m
        double wheelbase_;         // m
        double k_ld_;              // s   Ld = k_ld * v
        double min_ld_;            // m
        double max_ld_;            // m
        double curvature_gain_;    // Ld /= (1 + g * |kappa|)
        double curvature_window_;  // m   local curvature sampling window
        double a_lat_max_;         // m/s^2 maximum lateral acceleration
        double goal_tol_;          // m
        double slow_dist_;         // m
        double max_steer_;         // rad
        double path_timeout_;      // s
        double pose_timeout_;      // s
        double control_period_;    // s
        std::string global_frame_;
        std::string active_path_frame_;
        bool local_path_mode_{false};
        bool request_enable_{false};
        double command_validity_ms_{100.0};
        bool publish_visualization_{false};
        std::size_t max_path_points_{2000};
        uint64_t command_sequence_{0};
        uint64_t timing_sequence_{0};
        std::chrono::steady_clock::time_point expected_release_;
        bool release_initialized_{false};
        uint64_t input_received_count_{0};
        std::atomic<bool> stop_control_{false};
        std::thread control_thread_;
        std::mutex state_mutex_;
        double command_publish_ms_{0.0};

        // ------------------------------------------------------------------ //
        // State
        // ------------------------------------------------------------------ //
        std::vector<geometry_msgs::msg::PoseStamped> path_;  // path poses
        std::vector<Vec2> path_xy_;                          // (N,2) cache
        std::vector<double> segment_lengths_;
        bool has_path_{false};
        geometry_msgs::msg::Pose pose_;
        bool has_pose_{false};
        std::optional<rclcpp::Time> last_pose_time_;
        std::optional<rclcpp::Time> last_path_time_;
        double speed_{0.0};  // pose-derived speed [m/s], used for Ld
    };
} // namespace smart_car

#endif // PLANNER__CONTROLLER_HPP_

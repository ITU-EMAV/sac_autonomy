#include "controller.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <numeric>
#include <chrono>

namespace smart_car
{

    namespace
    {
        inline double norm(const std::array<double, 2> &v)
        {
            return std::hypot(v[0], v[1]);
        }

        inline std::array<double, 2> sub(const std::array<double, 2> &a,
                                         const std::array<double, 2> &b)
        {
            return {a[0] - b[0], a[1] - b[1]};
        }
    } // namespace

    Controller::Controller(const rclcpp::NodeOptions &options)
        : Node("controller_exe", options)
    {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // ------------------------------------------------------------------ //
        // Parameters
        // ------------------------------------------------------------------ //
        this->declare_parameter<double>("linear_velocity", 1.0);     // m/s
        this->declare_parameter<double>("min_linear_velocity", 1.0);  // m/s
        this->declare_parameter<double>("full_speed_path_length", 8.0);  // m
        this->declare_parameter<double>("wheelbase", 1.7);           // m
        this->declare_parameter<double>("k_ld", 1.2);                // s  Ld = k_ld * v
        // Keep this low or Ld will always clamp to the minimum at low speed.
        this->declare_parameter<double>("min_lookahead", 1.5);       // m
        this->declare_parameter<double>("max_lookahead", 6.0);       // m
        // A zero curvature gain uses curvature only for the speed limit.
        this->declare_parameter<double>("curvature_gain", 0.0);      // Ld /= (1+g*|kappa|)
        this->declare_parameter<double>("curvature_window", 3.0);    // m local curvature window
        this->declare_parameter<double>("a_lat_max", 0.5);           // m/s^2 max yanal ivme
        this->declare_parameter<double>("goal_tolerance", 0.5);      // m
        this->declare_parameter<double>("slow_down_distance", 3.0);  // m
        this->declare_parameter<double>("max_steer_deg", 23.0);      // deg (plant right limit)
        this->declare_parameter<double>("path_timeout", 50.0);       // s
        this->declare_parameter<double>("pose_timeout", 0.3);        // s
        this->declare_parameter<double>("control_period", 0.02);     // s (50 Hz)
        this->declare_parameter<std::string>("global_frame", "map");
        this->declare_parameter<bool>("local_path_mode", false);
        this->declare_parameter<std::string>("trajectory_topic", "/trajectory_planner/trajectory");
        this->declare_parameter<std::string>("pose_topic", "/pcl_pose");
        this->declare_parameter<std::string>("command_topic", "/control/controller_command");
        // Raw Twist output next to the guardian command. Set to "" to disable it.
        this->declare_parameter<std::string>("cmd_vel_topic", "");
        this->declare_parameter<bool>("request_enable", false);
        this->declare_parameter<double>("command_validity_ms", 100.0);
        this->declare_parameter<std::string>("lookahead_marker_topic", "/controller/lookahead");
        this->declare_parameter<bool>("publish_visualization", false);
        this->declare_parameter<int>("max_path_points", 2000);

        const bool require_hazard = declare_parameter<bool>("require_hazard_speed_constraint", false);
        const bool require_traffic = declare_parameter<bool>("require_traffic_speed_constraint", false);
        const bool require_map = declare_parameter<bool>("require_map_speed_constraint", false);
        const bool require_dynamic = declare_parameter<bool>("require_dynamic_speed_constraint", false);
        enforce_freshness_ = declare_parameter<bool>("enforce_freshness", true);
        constraint_timeout_ = declare_parameter<double>("hazard_speed_timeout", 0.3);
        if (!std::isfinite(constraint_timeout_) || constraint_timeout_ <= 0.0) {
            throw std::invalid_argument("hazard_speed_timeout must be finite and positive");
        }
        future_stamp_tolerance_ = declare_parameter<double>("future_stamp_tolerance", 0.1);
        if (!std::isfinite(future_stamp_tolerance_) || future_stamp_tolerance_ < 0.0) {
            throw std::invalid_argument("future_stamp_tolerance must be finite and nonnegative");
        }
        command_acceleration_ = declare_parameter<double>("max_command_acceleration", 0.5);
        command_deceleration_ = declare_parameter<double>("max_command_deceleration", 1.5);
        if (!std::isfinite(command_acceleration_) || command_acceleration_ <= 0.0 ||
            !std::isfinite(command_deceleration_) || command_deceleration_ <= 0.0) {
            throw std::invalid_argument("command acceleration/deceleration must be finite and positive");
        }
        linear_velocity_ = this->get_parameter("linear_velocity").as_double();
        min_linear_velocity_ = std::clamp(
            this->get_parameter("min_linear_velocity").as_double(), 0.0, linear_velocity_);
        full_speed_path_length_ = std::max(
            this->get_parameter("full_speed_path_length").as_double(), 1e-3);
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        k_ld_ = this->get_parameter("k_ld").as_double();
        min_ld_ = this->get_parameter("min_lookahead").as_double();
        max_ld_ = this->get_parameter("max_lookahead").as_double();
        curvature_gain_ = this->get_parameter("curvature_gain").as_double();
        curvature_window_ = this->get_parameter("curvature_window").as_double();
        a_lat_max_ = this->get_parameter("a_lat_max").as_double();
        goal_tol_ = this->get_parameter("goal_tolerance").as_double();
        slow_dist_ = this->get_parameter("slow_down_distance").as_double();
        max_steer_ = this->get_parameter("max_steer_deg").as_double() * M_PI / 180.0;
        path_timeout_ = this->get_parameter("path_timeout").as_double();
        pose_timeout_ = this->get_parameter("pose_timeout").as_double();
        if (!std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0)
        {
            throw std::invalid_argument("pose_timeout must be finite and positive");
        }
        control_period_ = this->get_parameter("control_period").as_double();
        global_frame_ = this->get_parameter("global_frame").as_string();
        active_path_frame_ = global_frame_;
        local_path_mode_ = this->get_parameter("local_path_mode").as_bool();

        const std::string TRAJECTORY_TOPIC = this->get_parameter("trajectory_topic").as_string();
        const std::string POSE_TOPIC = this->get_parameter("pose_topic").as_string();
        const std::string COMMAND_TOPIC = this->get_parameter("command_topic").as_string();
        const std::string CMD_VEL_TOPIC = this->get_parameter("cmd_vel_topic").as_string();
        request_enable_ = this->get_parameter("request_enable").as_bool();
        command_validity_ms_ = this->get_parameter("command_validity_ms").as_double();
        publish_visualization_ = this->get_parameter("publish_visualization").as_bool();
        max_path_points_ = static_cast<std::size_t>(std::max<int64_t>(
            2, this->get_parameter("max_path_points").as_int()));
        const std::string LOOKAHEAD_MARKER_TOPIC =
            this->get_parameter("lookahead_marker_topic").as_string();

        // ------------------------------------------------------------------ //
        // Subscriptions and publishers
        // ------------------------------------------------------------------ //
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;

        // All behavior producers share the same freshness and minimum-speed arbitration.
        const auto add_constraint = [this, &sub_options](const std::string & topic) {
            auto input = std::make_shared<SpeedInput>();
            input->topic = topic;
            input->subscription = create_subscription<sac_interfaces::msg::SpeedConstraint>(
                topic, rclcpp::QoS(1),
                [this, state = input.get()](const sac_interfaces::msg::SpeedConstraint & msg) {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
                    const double age = (get_clock()->now() - stamp).seconds();
                    // Repeated or reordered messages cannot replace the last valid cap.
                    if (state->stamp && stamp <= *state->stamp) return;
                    if (!std::isfinite(msg.max_speed_mps) || msg.max_speed_mps < 0.0 ||
                        (enforce_freshness_ &&
                         (age < -future_stamp_tolerance_ || age > constraint_timeout_))) {
                        state->speed = 0.0;
                        char buf[160];
                        std::snprintf(buf, sizeof(buf), "max_speed %.2f, age %.3fs, %s",
                            msg.max_speed_mps, age,
                            "invalid or late");
                        state->rejected = buf;
                        return;
                    }
                    state->rejected.clear();
                    state->stamp = stamp;
                    state->receipt = std::chrono::steady_clock::now();
                    state->speed = msg.max_speed_mps;
                    state->reason = msg.reason;
                }, sub_options);
            speed_inputs_.push_back(input);
        };
        if (require_hazard) add_constraint("/planning/road_hazard/speed_constraint");
        if (require_traffic) add_constraint("/planning/traffic/speed_constraint");
        if (require_map) add_constraint("/planning/map/speed_constraint");
        if (require_dynamic) add_constraint("/planning/dynamic/speed_constraint");

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            TRAJECTORY_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
            std::bind(&Controller::pathCallback, this, std::placeholders::_1), sub_options);

        // Reactive paths already describe the vehicle at the local origin.
        // A map-frame localization pose is neither needed nor valid in that frame.
        if (!local_path_mode_)
        {
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                POSE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
                std::bind(&Controller::locCallback, this, std::placeholders::_1), sub_options);
        }

        command_publisher_ = this->create_publisher<sac_interfaces::msg::ActuatorCommand>(
            COMMAND_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
        if (!CMD_VEL_TOPIC.empty())
        {
            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
                CMD_VEL_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
            RCLCPP_WARN(this->get_logger(),
                        "Publishing raw Twist on %s: this path bypasses command_guardian",
                        CMD_VEL_TOPIC.c_str());
        }
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            LOOKAHEAD_MARKER_TOPIC, 10);
        timing_publisher_ = this->create_publisher<sac_interfaces::msg::PipelineTiming>(
            "/control/timing", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
        // Latched so a late `ros2 topic echo` still sees the current state.
        state_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/control/controller_state", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

        control_thread_ = std::thread(&Controller::controlLoop, this);

        RCLCPP_INFO(
            this->get_logger(),
            "Pure pursuit controller started: frame=%s local_path_mode=%s trajectory=%s",
            global_frame_.c_str(), local_path_mode_ ? "true" : "false",
            TRAJECTORY_TOPIC.c_str());
    }

    Controller::~Controller()
    {
        stop_control_.store(true, std::memory_order_release);
        if (control_thread_.joinable()) control_thread_.join();
    }

    // ---------------------------------------------------------------------- //
    // Callbacks
    // ---------------------------------------------------------------------- //
    void Controller::pathCallback(const nav_msgs::msg::Path &msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++input_received_count_;
        if (msg.header.frame_id != global_frame_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Ignoring path in frame '%s'; expected '%s'.",
                        msg.header.frame_id.c_str(), global_frame_.c_str());
            return;
        }

        if (msg.poses.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "Empty/short path revoked the current trajectory.");
            has_path_ = false;
            path_.clear();
            return;
        }
        if (msg.poses.size() > max_path_points_)
        {
            RCLCPP_ERROR(this->get_logger(), "Rejecting path with %zu points; bounded limit is %zu.",
                         msg.poses.size(), max_path_points_);
            return;
        }

        path_ = msg.poses;
        // Markers always use the accepted path's TF frame. This is base_link in
        // reactive mode and map in mapped mode.
        active_path_frame_ = msg.header.frame_id;
        path_xy_.clear();
        path_xy_.reserve(msg.poses.size());
        for (const auto &p : msg.poses)
        {
            path_xy_.push_back({p.pose.position.x, p.pose.position.y});
        }
        segment_lengths_.resize(path_xy_.size() - 1);
        for (std::size_t i = 0; i + 1 < path_xy_.size(); ++i)
        {
            segment_lengths_[i] = norm(sub(path_xy_[i + 1], path_xy_[i]));
        }
        has_path_ = true;
        last_path_time_ = rclcpp::Time(msg.header.stamp, get_clock()->get_clock_type());
    }

    void Controller::locCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (msg.header.frame_id != global_frame_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring localization frame '%s'; expected '%s'.",
                                 msg.header.frame_id.c_str(), global_frame_.c_str());
            return;
        }
        if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring localization pose with zero timestamp.");
            return;
        }
        if (!std::isfinite(msg.pose.pose.position.x) ||
            !std::isfinite(msg.pose.pose.position.y) ||
            !std::isfinite(msg.pose.pose.position.z) ||
            !std::isfinite(msg.pose.pose.orientation.x) ||
            !std::isfinite(msg.pose.pose.orientation.y) ||
            !std::isfinite(msg.pose.pose.orientation.z) ||
            !std::isfinite(msg.pose.pose.orientation.w))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignoring non-finite localization position/orientation in frame '%s'.",
                                 msg.header.frame_id.c_str());
            return;
        }
        const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
        if (last_pose_time_ && stamp <= *last_pose_time_) return;
        // Estimate speed for adaptive lookahead from consecutive poses; a gap
        // over 0.5 s (dropout, relocalization) resets it to the fallback.
        const double dt = last_pose_time_ ? (stamp - *last_pose_time_).seconds() : 0.0;
        if (has_pose_ && dt > 0.0 && dt < 0.5)
        {
            const double v = std::hypot(msg.pose.pose.position.x - pose_.position.x,
                                        msg.pose.pose.position.y - pose_.position.y) / dt;
            speed_ = 0.7 * speed_ + 0.3 * v;
        }
        else
        {
            speed_ = 0.0;
        }
        pose_ = msg.pose.pose;
        last_pose_time_ = stamp;
        has_pose_ = true;
    }

    // ---------------------------------------------------------------------- //
    // Path curvature and projection
    // ---------------------------------------------------------------------- //
    Controller::Projection Controller::projectOnPath(const Vec2 &curr) const
    {
        // Nearest-segment projection, segment index, and remaining arc length.
        const auto &pts = path_xy_;
        const std::size_t n_seg = pts.size() - 1;

        std::size_t i0 = 0;
        double best_dist = std::numeric_limits<double>::max();
        Vec2 start = pts.front();

        for (std::size_t i = 0; i < n_seg; ++i)
        {
            const Vec2 ab = sub(pts[i + 1], pts[i]);
            const double seg_len2 = std::max(ab[0] * ab[0] + ab[1] * ab[1], 1e-9);

            const Vec2 ap = sub(curr, pts[i]);
            const double t = std::clamp((ap[0] * ab[0] + ap[1] * ab[1]) / seg_len2, 0.0, 1.0);
            const Vec2 proj = {pts[i][0] + t * ab[0], pts[i][1] + t * ab[1]};
            const double dist = norm(sub(proj, curr));

            if (dist < best_dist)
            {
                best_dist = dist;
                i0 = i;
                start = proj;
            }
        }

        double s_remain = norm(sub(pts[i0 + 1], start));
        for (std::size_t i = i0 + 1; i < n_seg; ++i)
        {
            s_remain += segment_lengths_[i];
        }

        return Projection{start, i0, s_remain};
    }

    double Controller::estimatePathCurvature(std::size_t i0, const Vec2 &start,
                                             double window_m) const
    {
        // Estimate path curvature (1/m) over window_m from the projection.
        if (path_xy_.size() < 3)
        {
            return 0.0;
        }

        const auto &pts = path_xy_;
        std::vector<Vec2> samples{start};
        double remaining = window_m;
        Vec2 p_prev = start;

        for (std::size_t i = i0; i + 1 < pts.size(); ++i)
        {
            const Vec2 p_next = pts[i + 1];
            const double seg = norm(sub(p_next, p_prev));
            if (seg < 1e-9)
            {
                continue;
            }
            if (seg >= remaining)
            {
                const double ratio = remaining / seg;
                samples.push_back({p_prev[0] + ratio * (p_next[0] - p_prev[0]),
                                   p_prev[1] + ratio * (p_next[1] - p_prev[1])});
                break;
            }
            samples.push_back(p_next);
            remaining -= seg;
            p_prev = p_next;
        }

        if (samples.size() < 3)
        {
            return 0.0;
        }

        double kappa_sum = 0.0;
        std::size_t kappa_count = 0;
        for (std::size_t j = 1; j + 1 < samples.size(); ++j)
        {
            const Vec2 &a = samples[j - 1];
            const Vec2 &b = samples[j];
            const Vec2 &c = samples[j + 1];
            const double ab = norm(sub(b, a));
            const double bc = norm(sub(c, b));
            const double ac = norm(sub(c, a));
            const double area2 =
                std::abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]));
            const double denom = ab * bc * ac;
            if (denom > 1e-9)
            {
                kappa_sum += 2.0 * area2 / denom;
                ++kappa_count;
            }
        }

        return kappa_count > 0 ? kappa_sum / static_cast<double>(kappa_count) : 0.0;
    }

    // ---------------------------------------------------------------------- //
    // Lookahead calculation
    // ---------------------------------------------------------------------- //
    std::optional<Controller::Lookahead> Controller::findLookaheadPoint(
        double Ld, const Projection &proj) const
    {
        // Find the point Ld ahead of the vehicle projection on the path.
        if (!has_path_ || (!local_path_mode_ && !has_pose_))
        {
            return std::nullopt;
        }

        const auto &pts = path_xy_;
        if (pts.size() < 2)
        {
            return Lookahead{pts.back(), 0, 0.0};
        }

        // Walk Ld from the projection and interpolate on the final segment.
        double remaining = Ld;
        Vec2 p_prev = proj.start;
        for (std::size_t i = proj.i0; i + 1 < pts.size(); ++i)
        {
            const Vec2 p_next = pts[i + 1];
            const double seg = norm(sub(p_next, p_prev));
            if (seg >= remaining)
            {
                const double ratio = seg > 1e-9 ? remaining / seg : 0.0;
                const Vec2 target = {p_prev[0] + ratio * (p_next[0] - p_prev[0]),
                                     p_prev[1] + ratio * (p_next[1] - p_prev[1])};
                return Lookahead{target, i + 1, proj.s_remain};
            }
            remaining -= seg;
            p_prev = p_next;
        }

        // Target the final pose when the remaining path is shorter than Ld.
        return Lookahead{pts.back(), pts.size() - 1, proj.s_remain};
    }

    // ---------------------------------------------------------------------- //
    // Control loop
    // ---------------------------------------------------------------------- //
    void Controller::controlLoop()
    {
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(control_period_));
        auto next_release = std::chrono::steady_clock::now() + period;
        while (!stop_control_.load(std::memory_order_acquire) && rclcpp::ok())
        {
            std::this_thread::sleep_until(next_release);
            if (stop_control_.load(std::memory_order_acquire) || !rclcpp::ok()) break;
            expected_release_ = next_release;
            release_initialized_ = true;
            controllerCallback();
            next_release += period;
            const auto now = std::chrono::steady_clock::now();
            if (now > next_release + period) next_release = now + period;
        }
    }

    void Controller::controllerCallback()
    {
        const auto callback_start = std::chrono::steady_clock::now();
        double jitter_ms = 0.0;
        if (release_initialized_)
        {
            jitter_ms = std::chrono::duration<double, std::milli>(
                callback_start - expected_release_).count();
        }
        const auto lock_start = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto lock_end = std::chrono::steady_clock::now();
        const double snapshot_lock_ms = std::chrono::duration<double, std::milli>(
            lock_end - lock_start).count();
        double path_projection_ms = 0.0;
        double control_compute_ms = 0.0;
        command_publish_ms_ = 0.0;
        const double source_age_ms = last_path_time_.has_value() ?
            (this->get_clock()->now() - last_path_time_.value()).seconds() * 1000.0 :
            std::numeric_limits<double>::infinity();
        const auto finish_timing = [&]() {
            publishTiming(jitter_ms,
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - callback_start).count(),
                source_age_ms, snapshot_lock_ms, path_projection_ms,
                control_compute_ms, command_publish_ms_);
        };
        // Input validation
        if (!has_path_ || path_.size() < 2)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No path available.");
            stop("no path");
            finish_timing();
            return;
        }

        if (!local_path_mode_ && !has_pose_)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No localization available.");
            stop("no localization");
            finish_timing();
            return;
        }

        if (!local_path_mode_ && last_pose_time_)
        {
            const double age = (this->get_clock()->now() - *last_pose_time_).seconds();
            if (age < -future_stamp_tolerance_ || age > pose_timeout_)
            {
                stop("stale localization pose", "age " + std::to_string(age) + " s");
                finish_timing();
                return;
            }
        }

        if (enforce_freshness_ && last_path_time_.has_value())
        {
            const double age = (this->get_clock()->now() - last_path_time_.value()).seconds();
            if (age < -future_stamp_tolerance_ || age > path_timeout_)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Path is stale (%.2fs); stopping.", age);
                stop("stale path", "age " + std::to_string(age) + " s");
                finish_timing();
                return;
            }
        }

        // Pose
        const auto &p = pose_.position;
        const double yaw = local_path_mode_ ? 0.0 : tf2::getYaw(pose_.orientation);
        const Vec2 curr = local_path_mode_ ? Vec2{0.0, 0.0} : Vec2{p.x, p.y};

        const auto projection_start = std::chrono::steady_clock::now();
        Projection proj = projectOnPath(curr);
        path_projection_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - projection_start).count();
        const auto compute_start = std::chrono::steady_clock::now();

        // Goal check
        if (proj.s_remain < goal_tol_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "GOAL REACHED!");
            stop("goal reached");
            if (publish_visualization_) publishLookaheadMarker(path_xy_.back(), curr, 0.0, true);
            finish_timing();
            return;
        }

        // Local path curvature (1/m) limits speed but not Ld when gain is zero.
        const double path_kappa = estimatePathCurvature(proj.i0, proj.start, curvature_window_);

        // Speed-adaptive lookahead; gain=0 makes Ld independent of curvature.
        // Ld = clip(k_ld * v, min_ld, max_ld)
        // Fall back to commanded speed below 0.15 m/s to reject pose-derived speed noise.
        const double v_for_ld = speed_ > 0.15 ? speed_ : linear_velocity_;
        double Ld_cmd = k_ld_ * v_for_ld;
        if (curvature_gain_ > 0.0 && std::abs(path_kappa) > 1e-6)
        {
            Ld_cmd /= 1.0 + curvature_gain_ * std::abs(path_kappa);
        }
        Ld_cmd = std::clamp(Ld_cmd, min_ld_, max_ld_);

        const auto res = findLookaheadPoint(Ld_cmd, proj);
        if (!res.has_value())
        {
            stop("no lookahead point on path");
            finish_timing();
            return;
        }
        const Vec2 target = res->target;
        const double s_remain = res->s_remain;

        // --- Pure pursuit ---
        const Vec2 d = sub(target, curr);
        const double Ld = std::hypot(d[0], d[1]);
        if (Ld < 1e-3)
        {
            stop("lookahead point too close");
            finish_timing();
            return;
        }

        double alpha = std::atan2(d[1], d[0]) - yaw;
        alpha = std::atan2(std::sin(alpha), std::cos(alpha)); // [-pi, pi]

        double kappa = 2.0 * std::sin(alpha) / Ld;

        // Ackermann steering limit: kappa_max = tan(delta_max) / L
        const double kappa_max = std::tan(max_steer_) / wheelbase_;
        kappa = std::clamp(kappa, -kappa_max, kappa_max);

        const auto target_now = std::chrono::steady_clock::now();
        const double target_dt = std::clamp(std::chrono::duration<double>(
            target_now - previous_target_time_).count(), 0.0, 2.0 * control_period_);

        // Speed profile
        // 1) A rolling reactive path is not a final goal. Scale its reference
        // speed between the configured minimum and maximum using visible path length.
        double v;
        if (local_path_mode_)
        {
            const double ratio = std::clamp(s_remain / full_speed_path_length_, 0.0, 1.0);
            v = min_linear_velocity_ +
                (linear_velocity_ - min_linear_velocity_) * ratio;
            // Visible local-path length is a rolling reference, not a hard cap.
            // Smooth decreases before applying curvature and behavior limits.
            v = std::max(v, previous_target_speed_ -
                0.9 * command_deceleration_ * target_dt);
        }
        else
        {
            v = linear_velocity_ * std::min(1.0, s_remain / std::max(slow_dist_, 1e-3));
        }
        // 2) Lateral acceleration limit: a_lat = v^2 * |kappa| <= a_lat_max
        //    => v_max = sqrt(a_lat_max / |kappa|)
        std::string limiting_source = local_path_mode_ ? "local path reference" : "goal approach";
        if (std::abs(path_kappa) > 1e-6)
        {
            const double curvature_cap = std::sqrt(a_lat_max_ / std::abs(path_kappa));
            if (curvature_cap < v) {
                v = curvature_cap;
                limiting_source = "path curvature";
            }
        }
        // A nominal minimum must never override curvature or behavior constraints.
        v = std::clamp(v, 0.0, linear_velocity_);
        for (const auto & input : speed_inputs_) {
            const double wall_age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - input->receipt).count();
            const double age = input->stamp ?
                (get_clock()->now() - *input->stamp).seconds() : -1.0;
            if (!input->stamp || (enforce_freshness_ &&
                (age < -future_stamp_tolerance_ || age > constraint_timeout_ ||
                 wall_age > constraint_timeout_))) {
                char buf[160];
                std::snprintf(buf, sizeof(buf), "age %.3fs, receipt age %.3fs, timeout %.3fs",
                    age, wall_age, constraint_timeout_);
                stop("speed constraint " + input->topic +
                    (input->stamp ? " stale" : " never received"), buf);
                finish_timing();
                return;
            }
            if (input->speed < v) {
                v = input->speed;
                limiting_source = input->topic;
            }
        }
        if (v <= 0.0) {
            std::string reason = "zero target speed";
            std::string detail;
            for (const auto & input : speed_inputs_) {
                if (input->speed <= 0.0) {
                    reason = "speed constraint " + input->topic +
                        (input->rejected.empty() ? " max_speed 0" : " rejected message");
                    detail = input->rejected.empty() ? input->reason : input->rejected;
                    break;
                }
            }
            stop(reason, detail);
            finish_timing();
            return;
        }

        // Ramp positive speed requests. A suddenly infeasible lower safety cap
        // must stop the vehicle, never be relaxed merely to satisfy a slew limit.
        if (previous_target_speed_ - v > command_deceleration_ * target_dt) {
            char buf[160];
            std::snprintf(buf, sizeof(buf), "%.2f -> %.2f m/s in %.3fs, max %.2f m/s^2",
                previous_target_speed_, v, target_dt, command_deceleration_);
            stop("target speed drop exceeds max deceleration", limiting_source + ": " + buf);
            finish_timing();
            return;
        }
        v = std::min(v, previous_target_speed_ + 0.9 * command_acceleration_ * target_dt);
        previous_target_speed_ = v;
        previous_target_time_ = target_now;

        sac_interfaces::msg::ActuatorCommand command;
        command.source_stamp = this->get_clock()->now();
        command.publication_stamp = command.source_stamp;
        command.source_steady_time_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        command.sequence = ++command_sequence_;
        command.validity_duration.sec = 0;
        command.validity_duration.nanosec = static_cast<uint32_t>(
            std::clamp(command_validity_ms_, 1.0, 999.0) * 1e6);
        command.mode = sac_interfaces::msg::ActuatorCommand::MODE_AUTONOMOUS;
        command.armed = request_enable_;
        command.enable = request_enable_;
        command.steering_angle_rad = std::atan(wheelbase_ * kappa);
        command.steering_rate_radps = 0.0;
        command.throttle_normalized = 0.0;
        command.drive_torque_nm = 0.0;
        command.brake_normalized = 0.0;
        command.target_speed_mps = v;
        command.target_acceleration_mps2 = 0.0;
        command.validity_flags = sac_interfaces::msg::ActuatorCommand::VALID_STEERING |
            sac_interfaces::msg::ActuatorCommand::VALID_THROTTLE |
            sac_interfaces::msg::ActuatorCommand::VALID_BRAKE |
            sac_interfaces::msg::ActuatorCommand::VALID_TARGET_SPEED |
            sac_interfaces::msg::ActuatorCommand::INTEGRITY_OK;
        command.source_id = "planner_controller";
        control_compute_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - compute_start).count();
        const auto publish_start = std::chrono::steady_clock::now();
        command_publisher_->publish(command);
        reportState("DRIVING");
        if (cmd_vel_publisher_)
        {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = v;
            // Twist carries the steering angle, not a yaw rate, as in the Python tracker.
            twist.angular.z = command.steering_angle_rad;
            cmd_vel_publisher_->publish(twist);
        }
        command_publish_ms_ = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - publish_start).count();

        if (publish_visualization_) publishLookaheadMarker(target, curr, Ld);
        const auto callback_end = std::chrono::steady_clock::now();
        publishTiming(jitter_ms,
            std::chrono::duration<double, std::milli>(callback_end - callback_start).count(),
            source_age_ms, snapshot_lock_ms, path_projection_ms,
            control_compute_ms, command_publish_ms_);
    }

    void Controller::publishTiming(double jitter_ms, double execution_ms, double source_age_ms,
                                   double snapshot_lock_ms, double path_projection_ms,
                                   double control_compute_ms, double publish_ms)
    {
        sac_interfaces::msg::PipelineTiming timing;
        timing.stamp = this->get_clock()->now();
        timing.component = "controller";
        timing.sequence = ++timing_sequence_;
        timing.release_jitter_ms = jitter_ms;
        timing.execution_time_ms = execution_ms;
        timing.source_age_ms = source_age_ms;
        timing.queue_delay_ms = 0.0;
        timing.snapshot_lock_ms = snapshot_lock_ms;
        timing.path_projection_ms = path_projection_ms;
        timing.control_compute_ms = control_compute_ms;
        timing.publish_ms = publish_ms;
        timing.received_count = input_received_count_;
        timing.processed_count = command_sequence_;
        timing_publisher_->publish(timing);
    }

    void Controller::reportState(const std::string &state, const std::string &detail)
    {
        // Compare only the state, so per-tick numbers in detail do not flood the log.
        if (state == last_state_) return;
        const std::string text = detail.empty() ? state : state + " (" + detail + ")";
        RCLCPP_WARN(this->get_logger(), "Controller state: %s -> %s",
                    last_state_.empty() ? "START" : last_state_.c_str(), text.c_str());
        last_state_ = state;
        std_msgs::msg::String msg;
        msg.data = text;
        state_publisher_->publish(msg);
    }

    void Controller::stop(const std::string &reason, const std::string &detail)
    {
        reportState("STOP: " + reason, detail);
        previous_target_speed_ = 0.0;
        previous_target_time_ = std::chrono::steady_clock::now();
        sac_interfaces::msg::ActuatorCommand command;
        command.source_stamp = this->get_clock()->now();
        command.publication_stamp = command.source_stamp;
        command.source_steady_time_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        command.sequence = ++command_sequence_;
        command.validity_duration.nanosec = static_cast<uint32_t>(
            std::clamp(command_validity_ms_, 1.0, 999.0) * 1e6);
        command.mode = sac_interfaces::msg::ActuatorCommand::MODE_CONTROLLED_STOP;
        command.brake_normalized = 1.0;
        command.validity_flags = sac_interfaces::msg::ActuatorCommand::VALID_STEERING |
            sac_interfaces::msg::ActuatorCommand::VALID_THROTTLE |
            sac_interfaces::msg::ActuatorCommand::VALID_BRAKE |
            sac_interfaces::msg::ActuatorCommand::VALID_TARGET_SPEED |
            sac_interfaces::msg::ActuatorCommand::INTEGRITY_OK;
        command.source_id = "planner_controller";
        const auto publish_start = std::chrono::steady_clock::now();
        command_publisher_->publish(command);
        if (cmd_vel_publisher_) cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
        command_publish_ms_ = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - publish_start).count();
    }

    // ---------------------------------------------------------------------- //
    // Visualization
    // ---------------------------------------------------------------------- //
    void Controller::publishLookaheadMarker(const Vec2 &target, const Vec2 &curr, double Ld,
                                            bool reached)
    {
        visualization_msgs::msg::MarkerArray ma;
        const auto stamp = this->get_clock()->now();

        auto base = [&](int mid, int mtype)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = active_path_frame_;
            m.header.stamp = stamp;
            m.ns = "lookahead";
            m.id = mid;
            m.type = mtype;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.lifetime.sec = 0;
            m.lifetime.nanosec = 500000000; // 0.5 s
            return m;
        };

        // 1) Lookahead point: red sphere, or green when the goal is reached.
        auto sphere = base(0, visualization_msgs::msg::Marker::SPHERE);
        sphere.pose.position.x = target[0];
        sphere.pose.position.y = target[1];
        sphere.pose.position.z = 1.5;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5;
        if (reached)
        {
            sphere.color.r = 0.1;
            sphere.color.g = 1.0;
            sphere.color.b = 0.1;
        }
        else
        {
            sphere.color.r = 1.0;
            sphere.color.g = 0.1;
            sphere.color.b = 0.1;
        }
        sphere.color.a = 1.0;
        ma.markers.push_back(sphere);

        // 2) Vehicle-to-lookahead vector: green line.
        auto line = base(1, visualization_msgs::msg::Marker::LINE_STRIP);
        line.scale.x = 0.08;
        line.color.r = 0.1;
        line.color.g = 1.0;
        line.color.b = 0.3;
        line.color.a = 0.9;
        for (const Vec2 &xy : {curr, target})
        {
            geometry_msgs::msg::Point pt;
            pt.x = xy[0];
            pt.y = xy[1];
            pt.z = 0.2;
            line.points.push_back(pt);
        }
        ma.markers.push_back(line);

        // 3) Lookahead circle: blue, vehicle-centered, radius Ld.
        auto circle = base(2, visualization_msgs::msg::Marker::LINE_STRIP);
        circle.scale.x = 0.04;
        circle.color.r = 0.2;
        circle.color.g = 0.5;
        circle.color.b = 1.0;
        circle.color.a = 0.6;
        constexpr int kCircleSamples = 48;
        for (int i = 0; i < kCircleSamples; ++i)
        {
            const double th = 2.0 * M_PI * static_cast<double>(i) / (kCircleSamples - 1);
            geometry_msgs::msg::Point pt;
            pt.x = curr[0] + Ld * std::cos(th);
            pt.y = curr[1] + Ld * std::sin(th);
            pt.z = 0.05;
            circle.points.push_back(pt);
        }
        ma.markers.push_back(circle);

        // 4) Ld value label.
        auto text = base(3, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        text.pose.position.x = target[0];
        text.pose.position.y = target[1];
        text.pose.position.z = 0.9;
        text.scale.z = 0.4;
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
        char buf[32];
        std::snprintf(buf, sizeof(buf), "Ld = %.2f m", Ld);
        text.text = buf;
        ma.markers.push_back(text);

        marker_publisher_->publish(ma);
    }

} // namespace smart_car

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(smart_car::Controller)

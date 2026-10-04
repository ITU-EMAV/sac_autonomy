#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "cluster_msgs/msg/cluster_points_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sac_interfaces/msg/actuator_command.hpp"
#include "sac_interfaces/msg/camera_detection_array.hpp"
#include "sac_interfaces/msg/command_guardian_status.hpp"
#include "sac_interfaces/msg/vehicle_speed.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

namespace
{
uint64_t steady_now_ns()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch()).count());
}

double duration_seconds(const builtin_interfaces::msg::Duration & value)
{
  return static_cast<double>(value.sec) + static_cast<double>(value.nanosec) * 1e-9;
}
}  // namespace

class CommandGuardian final : public rclcpp::Node
{
public:
  using Command = sac_interfaces::msg::ActuatorCommand;
  using Status = sac_interfaces::msg::CommandGuardianStatus;

  CommandGuardian()
  : Node("command_guardian")
  {
    enabled_ = declare_parameter("enabled", true);
    input_topic_ = declare_parameter("controller_input_topic", "/control/controller_command");
    output_topic_ = declare_parameter("actuator_output_topic", "/vehicle/actuator_command");
    expected_source_id_ = declare_parameter("expected_source_id", "planner_controller");
    allow_vehicle_output_ = declare_parameter("allow_vehicle_output", false);
    require_collision_ = declare_parameter("require_collision_perception", true);
    require_camera_ = declare_parameter("require_camera_behavior", false);
    require_localization_ = declare_parameter("require_localization", true);
    feedback_age_ = ms("vehicle_feedback_max_age_ms", 50.0);
    localization_age_ = ms("localization_max_age_ms", 100.0);
    trajectory_age_ = ms("trajectory_max_age_ms", 250.0);
    collision_age_ = ms("collision_perception_max_age_ms", 200.0);
    camera_age_ = ms("camera_behavior_max_age_ms", 300.0);
    command_age_ = ms("command_max_age_ms", 100.0);
    future_tolerance_ = ms("future_tolerance_ms", 5.0);
    max_steer_ = declare_parameter("max_steering_angle_rad", 0.40143);
    max_steer_rate_ = declare_parameter("max_steering_rate_radps", 0.35);
    max_throttle_ = declare_parameter("max_throttle_normalized", 0.30);
    max_torque_ = declare_parameter("max_drive_torque_nm", 0.0);
    max_brake_ = declare_parameter("max_brake_normalized", 1.0);
    max_speed_ = declare_parameter("max_target_speed_mps", 2.0);
    max_accel_ = declare_parameter("max_acceleration_mps2", 0.5);
    max_decel_ = declare_parameter("max_deceleration_mps2", 1.5);
    safe_brake_ = declare_parameter("safe_stop_brake_normalized", 1.0);
    const int output_period_ms = declare_parameter("output_period_ms", 20);

    if (!enabled_) {
      // Stay inert: no subscriptions, no timer, nothing published on the actuator topic.
      RCLCPP_WARN(
        get_logger(),
        "command_guardian is DISABLED by configuration (enabled=false): no input is validated "
        "and nothing is published on %s", output_topic_.c_str());
      return;
    }

    const auto latest_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    command_sub_ = create_subscription<Command>(
      input_topic_, latest_qos,
      [this](Command::ConstSharedPtr msg) {command_callback(*msg);});
    command_pub_ = create_publisher<Command>(output_topic_, latest_qos);
    status_pub_ = create_publisher<Status>("/safety/command_guardian/status", latest_qos);

    localization_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter("localization_topic", "/localization/online/pose"), latest_qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
        localization_stamp_ = rclcpp::Time(msg.header.stamp); localization_received_ = true;
      });
    feedback_sub_ = create_subscription<sac_interfaces::msg::VehicleSpeed>(
      declare_parameter("vehicle_feedback_topic", "/vehicle/speed"), latest_qos,
      [this](const sac_interfaces::msg::VehicleSpeed & msg) {
        feedback_stamp_ = rclcpp::Time(msg.header.stamp); feedback_received_ = true;
      });
    trajectory_sub_ = create_subscription<nav_msgs::msg::Path>(
      declare_parameter("trajectory_topic", "/planning/trajectory"), latest_qos,
      [this](const nav_msgs::msg::Path & msg) {
        trajectory_stamp_ = rclcpp::Time(
          msg.header.stamp); trajectory_received_ = !msg.poses.empty();
      });
    collision_sub_ = create_subscription<cluster_msgs::msg::ClusterPointsArray>(
      declare_parameter("collision_perception_topic", "/perception/lidar/clusters"), latest_qos,
      [this](const cluster_msgs::msg::ClusterPointsArray & msg) {
        collision_stamp_ = rclcpp::Time(msg.header.stamp); collision_received_ = true;
      });
    camera_sub_ = create_subscription<sac_interfaces::msg::CameraDetectionArray>(
      declare_parameter("camera_behavior_topic", "/perception/camera/detections"), latest_qos,
      [this](const sac_interfaces::msg::CameraDetectionArray & msg) {
        camera_stamp_ = rclcpp::Time(msg.header.stamp); camera_received_ = true;
      });

    arm_service_ = create_service<std_srvs::srv::SetBool>(
      "/safety/command_guardian/arm",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
        armed_by_operator_ = request->data;
        if (!armed_by_operator_) {
          state_ = Status::STATE_DISABLED;
          fault_reason_ = "operator disarmed";
          have_command_ = false;
          fault_latched_ = false;
        } else if (!allow_vehicle_output_) {
          state_ = Status::STATE_ARMED;
          fault_reason_ = "armed for test but vehicle output is administratively disabled";
        } else {
          state_ = Status::STATE_ARMED;
          fault_reason_.clear();
        }
        response->success = true;
        response->message = fault_reason_;
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(output_period_ms, 5)),
      [this]() {evaluate_and_publish();});
    RCLCPP_WARN(
      get_logger(), "Guardian started fail-closed: allow_vehicle_output=%s",
      allow_vehicle_output_ ? "true" : "false");
    if (!require_localization_) {
      RCLCPP_WARN(
        get_logger(),
        "require_localization=false: the localization freshness gate is DISABLED; "
        "commands pass with absent or stale localization");
    }
  }

private:
  std::chrono::nanoseconds ms(const std::string & name, double default_ms)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(declare_parameter(name, default_ms)));
  }

  bool finite_and_bounded(const Command & c, std::string & reason) const
  {
    const double values[] = {c.steering_angle_rad, c.steering_rate_radps,
      c.throttle_normalized, c.drive_torque_nm, c.brake_normalized,
      c.target_speed_mps, c.target_acceleration_mps2};
    for (const double value : values) {
      if (!std::isfinite(value)) {reason = "NaN or infinite command field"; return false;}
    }
    if (std::abs(c.steering_angle_rad) > max_steer_ ||
      std::abs(c.steering_rate_radps) > max_steer_rate_ ||
      c.throttle_normalized < 0.0 || c.throttle_normalized > max_throttle_ ||
      std::abs(c.drive_torque_nm) > max_torque_ ||
      c.brake_normalized < 0.0 || c.brake_normalized > max_brake_ ||
      c.target_speed_mps < 0.0 || c.target_speed_mps > max_speed_ ||
      c.target_acceleration_mps2 > max_accel_ ||
      c.target_acceleration_mps2 < -max_decel_)
    {
      reason = "command exceeds configured physical limit"; return false;
    }
    const uint32_t required = Command::VALID_STEERING | Command::VALID_THROTTLE |
      Command::VALID_BRAKE | Command::VALID_TARGET_SPEED | Command::INTEGRITY_OK;
    if ((c.validity_flags & required) != required) {
      reason = "required validity/integrity flags are not set"; return false;
    }
    if (c.throttle_normalized > 0.0 && c.brake_normalized > 0.0) {
      reason = "simultaneous throttle and brake request"; return false;
    }
    return true;
  }

  void reject(const std::string & reason)
  {
    ++rejected_count_;
    fault_reason_ = reason;
    state_ = Status::STATE_FAULT;
    fault_latched_ = true;
    have_command_ = false;
  }

  void command_callback(const Command & c)
  {
    if (command_sub_->get_publisher_count() != 1U) {
      reject("controller topic must have exactly one publisher"); return;
    }
    if (c.source_id != expected_source_id_) {reject("unexpected controller source_id"); return;}
    if (have_sequence_ && c.sequence <= last_accepted_sequence_) {
      reject("duplicate or out-of-order sequence"); return;
    }
    const rclcpp::Time ros_source(c.source_stamp);
    if (ros_source.nanoseconds() <= 0) {reject("missing ROS source timestamp"); return;}
    const auto ros_age = get_clock()->now() - ros_source;
    if (ros_age.nanoseconds() < -future_tolerance_.count()) {
      reject("future-dated ROS command timestamp"); return;
    }
    if (ros_age.nanoseconds() > command_age_.count()) {
      reject("stale ROS command timestamp"); return;
    }
    if (c.source_steady_time_ns == 0U) {reject("missing monotonic source timestamp"); return;}
    const uint64_t now_ns = steady_now_ns();
    if (c.source_steady_time_ns > now_ns + static_cast<uint64_t>(future_tolerance_.count())) {
      reject("future-dated monotonic command"); return;
    }
    const uint64_t age_ns = now_ns - std::min(now_ns, c.source_steady_time_ns);
    const double validity_s = duration_seconds(c.validity_duration);
    if (!(validity_s > 0.0) || age_ns > static_cast<uint64_t>(validity_s * 1e9) ||
      age_ns > static_cast<uint64_t>(command_age_.count()))
    {
      reject("stale or expired controller command"); return;
    }
    std::string reason;
    if (!finite_and_bounded(c, reason)) {reject(reason); return;}
    if (have_command_) {
      if (c.source_steady_time_ns <= last_command_.source_steady_time_ns) {
        reject("non-increasing command monotonic time"); return;
      }
      const double dt = static_cast<double>(c.source_steady_time_ns -
        last_command_.source_steady_time_ns) * 1e-9;
      if (std::abs(c.steering_angle_rad - last_command_.steering_angle_rad) / dt >
        max_steer_rate_) {reject("measured steering command slew exceeds limit"); return;}
      const double acceleration = (c.target_speed_mps - last_command_.target_speed_mps) / dt;
      if (acceleration > max_accel_ || acceleration < -max_decel_) {
        reject("target-speed acceleration/deceleration exceeds limit"); return;
      }
    }
    last_command_ = c;
    last_command_receive_ns_ = now_ns;
    last_accepted_sequence_ = c.sequence;
    have_sequence_ = true;
    have_command_ = true;
  }

  bool fresh(
    bool received, const rclcpp::Time & stamp,
    std::chrono::nanoseconds limit, const char * name, std::string & reason)
  {
    if (!received || stamp.nanoseconds() <= 0) {
      reason = std::string("missing ") + name; return false;
    }
    const rclcpp::Time now = get_clock()->now();
    const auto delta = now - stamp;
    if (delta.nanoseconds() < -future_tolerance_.count()) {
      reason = std::string("future-dated ") + name; return false;
    }
    if (delta.nanoseconds() > limit.count()) {reason = std::string("stale ") + name; return false;}
    return true;
  }

  Command safe_command(uint8_t mode)
  {
    Command out;
    out.source_stamp = get_clock()->now();
    out.publication_stamp = out.source_stamp;
    out.source_steady_time_ns = steady_now_ns();
    out.sequence = output_sequence_;
    out.validity_duration.sec = 0;
    out.validity_duration.nanosec = 100000000U;
    out.mode = mode;
    out.armed = false;
    out.enable = false;
    out.brake_normalized = std::clamp(safe_brake_, 0.0, max_brake_);
    out.validity_flags = Command::VALID_STEERING | Command::VALID_THROTTLE |
      Command::VALID_BRAKE | Command::VALID_TARGET_SPEED | Command::INTEGRITY_OK;
    out.source_id = "command_guardian";
    return out;
  }

  void evaluate_and_publish()
  {
    ++output_sequence_;
    std::string reason;
    const uint64_t now_ns = steady_now_ns();
    const bool command_live = have_command_ &&
      now_ns - last_command_receive_ns_ <= static_cast<uint64_t>(command_age_.count()) &&
      now_ns - last_command_.source_steady_time_ns <=
      static_cast<uint64_t>(duration_seconds(last_command_.validity_duration) * 1e9);
    bool inputs_live = !require_localization_ || fresh(
      localization_received_, localization_stamp_, localization_age_,
      "localization", reason);
    inputs_live = inputs_live &&
      fresh(feedback_received_, feedback_stamp_, feedback_age_, "vehicle feedback", reason) &&
      fresh(trajectory_received_, trajectory_stamp_, trajectory_age_, "trajectory", reason);
    if (inputs_live && require_collision_) {
      inputs_live = fresh(
        collision_received_, collision_stamp_, collision_age_,
        "collision perception", reason);
    }
    if (inputs_live && require_camera_) {
      inputs_live = fresh(
        camera_received_, camera_stamp_, camera_age_,
        "camera behavior detections", reason);
    }

    Command output = safe_command(Command::MODE_CONTROLLED_STOP);
    const bool permitted = allow_vehicle_output_ && armed_by_operator_ && command_live &&
      inputs_live &&
      last_command_.armed && last_command_.enable && last_command_.mode == Command::MODE_AUTONOMOUS;
    if (fault_latched_) {
      state_ = Status::STATE_FAULT;
      output.mode = Command::MODE_EMERGENCY_STOP;
    } else if (permitted) {
      output = last_command_;
      output.publication_stamp = get_clock()->now();
      output.sequence = output_sequence_;
      output.source_id = "command_guardian";
      state_ = Status::STATE_ACTIVE;
      fault_reason_.clear();
    } else if (!armed_by_operator_) {
      state_ = Status::STATE_DISABLED;
      fault_reason_ = "operator not armed";
      output.mode = Command::MODE_DISABLED;
    } else if (!allow_vehicle_output_) {
      state_ = Status::STATE_ARMED;
      fault_reason_ = "vehicle output administratively disabled";
    } else if (!command_live) {
      state_ = Status::STATE_STOPPING;
      fault_reason_ = "missing or expired controller command";
    } else if (!inputs_live) {
      state_ = Status::STATE_STOPPING;
      fault_reason_ = reason;
    } else {
      state_ = Status::STATE_ARMED;
      fault_reason_ = "controller has not requested enable";
    }
    command_pub_->publish(output);

    Status status;
    status.stamp = get_clock()->now();
    status.state = state_;
    status.healthy = state_ != Status::STATE_FAULT;
    status.transport_enabled = permitted;
    status.fault_reason = fault_reason_;
    status.command_age_ms = have_command_ ? static_cast<double>(
      now_ns - last_command_.source_steady_time_ns) / 1e6 : std::numeric_limits<double>::infinity();
    status.last_accepted_sequence = last_accepted_sequence_;
    status.rejected_command_count = rejected_count_;
    status_pub_->publish(status);
  }

  std::string input_topic_, output_topic_, expected_source_id_, fault_reason_;
  bool allow_vehicle_output_{false}, require_collision_{true}, require_camera_{false};
  bool require_localization_{true}, enabled_{true};
  bool armed_by_operator_{false}, have_command_{false}, have_sequence_{false};
  bool fault_latched_{false};
  bool localization_received_{false}, feedback_received_{false}, trajectory_received_{false};
  bool collision_received_{false}, camera_received_{false};
  uint8_t state_{Status::STATE_DISABLED};
  uint64_t last_accepted_sequence_{0}, rejected_count_{0}, output_sequence_{0};
  uint64_t last_command_receive_ns_{0};
  double max_steer_, max_steer_rate_, max_throttle_, max_torque_, max_brake_, max_speed_;
  double max_accel_, max_decel_, safe_brake_;
  std::chrono::nanoseconds feedback_age_, localization_age_, trajectory_age_, collision_age_;
  std::chrono::nanoseconds camera_age_, command_age_, future_tolerance_;
  rclcpp::Time localization_stamp_{0}, feedback_stamp_{0}, trajectory_stamp_{0};
  rclcpp::Time collision_stamp_{0}, camera_stamp_{0};
  Command last_command_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Publisher<Command>::SharedPtr command_pub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr localization_sub_;
  rclcpp::Subscription<sac_interfaces::msg::VehicleSpeed>::SharedPtr feedback_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<cluster_msgs::msg::ClusterPointsArray>::SharedPtr collision_sub_;
  rclcpp::Subscription<sac_interfaces::msg::CameraDetectionArray>::SharedPtr camera_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandGuardian>());
  rclcpp::shutdown();
  return 0;
}

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sac_interfaces/msg/actuator_command.hpp"

// Fail-closed Phase-1 boundary. The old "@16 ..." datagrams could not carry
// a coherent command, sequence, timestamp, deadline, or integrity state.
class UdpActuatorTransport final : public rclcpp::Node
{
public:
  using Command = sac_interfaces::msg::ActuatorCommand;
  UdpActuatorTransport() : Node("udp_actuator_transport")
  {
    enabled_ = declare_parameter("transport_enabled", false);
    gateway_contract_ = declare_parameter("gateway_supports_safety_contract", false);
    legacy_cmd_vel_enabled_ = declare_parameter("legacy_cmd_vel_enabled", false);
    legacy_max_speed_mps_ = declare_parameter("legacy_max_speed_mps", 2.0);
    legacy_max_steering_rad_ = declare_parameter("legacy_max_steering_rad", 0.40143);
    address_ = declare_parameter("gateway_address", "10.42.0.214");
    port_ = declare_parameter("gateway_port", 4950);
    max_age_ms_ = declare_parameter("max_command_age_ms", 100.0);
    socket_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (socket_ < 0) {throw std::runtime_error("failed to create UDP socket");}
    std::memset(&gateway_, 0, sizeof(gateway_));
    gateway_.sin_family = AF_INET;
    gateway_.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, address_.c_str(), &gateway_.sin_addr) != 1) {
      throw std::runtime_error("invalid gateway_address");
    }
    if (legacy_cmd_vel_enabled_) {
      if (enabled_ || gateway_contract_ || !std::isfinite(legacy_max_speed_mps_) ||
        legacy_max_speed_mps_ <= 0.0 || !std::isfinite(legacy_max_steering_rad_) ||
        legacy_max_steering_rad_ <= 0.0) {
        throw std::runtime_error("invalid legacy cmd_vel transport configuration");
      }
      legacy_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(1).reliable().durability_volatile(),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_legacy_cmd_vel(*msg);});
      legacy_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (legacy_seen_ && std::chrono::steady_clock::now() - legacy_last_received_ >
          std::chrono::milliseconds(200)) {
          send_legacy(0.0, 0.0);
        }
      });
      RCLCPP_WARN(get_logger(), "Legacy /cmd_vel UDP output ENABLED to %s:%d",
        address_.c_str(), port_);
      return;
    }
    sub_ = create_subscription<Command>(
      "/vehicle/actuator_command", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](Command::ConstSharedPtr msg) {on_command(*msg);});
    if (!enabled_) {
      RCLCPP_WARN(get_logger(), "Vehicle UDP output is DISABLED (dry run)");
    } else if (!gateway_contract_) {
      RCLCPP_FATAL(get_logger(), "Refusing output: gateway safety contract is not commissioned");
      enabled_ = false;
    }
  }
  ~UdpActuatorTransport() override
  {
    if (legacy_cmd_vel_enabled_ && legacy_seen_) {send_legacy(0.0, 0.0);}
    if (socket_ >= 0) {close(socket_);}
  }

private:
  static std::string legacy_packet(const unsigned register_id, const double value, const double scale)
  {
    const auto fixed = static_cast<int32_t>(std::trunc(value * scale));
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "@16 %u %08X", register_id,
      static_cast<uint32_t>(fixed));
    return buffer;
  }
  void send_legacy(const double speed_mps, const double steering_rad)
  {
    const auto speed = legacy_packet(21U, speed_mps, 32.0);
    const auto steer = legacy_packet(22U, steering_rad, 16384.0);
    for (const auto & packet : {speed, steer}) {
      const auto sent = sendto(socket_, packet.data(), packet.size(), MSG_DONTWAIT,
        reinterpret_cast<const sockaddr *>(&gateway_), sizeof(gateway_));
      if (sent != static_cast<ssize_t>(packet.size())) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Legacy UDP send failed");
      }
    }
  }
  void on_legacy_cmd_vel(const geometry_msgs::msg::Twist & msg)
  {
    if (legacy_sub_->get_publisher_count() != 1U || !std::isfinite(msg.linear.x) ||
      !std::isfinite(msg.angular.z)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "Rejected legacy /cmd_vel: expected one publisher and finite values");
      return;
    }
    legacy_last_received_ = std::chrono::steady_clock::now();
    legacy_seen_ = true;
    send_legacy(std::clamp(msg.linear.x, -legacy_max_speed_mps_, legacy_max_speed_mps_),
      std::clamp(msg.angular.z, -legacy_max_steering_rad_, legacy_max_steering_rad_));
  }
  static uint64_t steady_now_ns()
  {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  }
  void on_command(const Command & command)
  {
    if (sub_->get_publisher_count() != 1U || command.source_id != "command_guardian") {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Rejected: command guardian must be the sole publisher");
      return;
    }
    const uint64_t now_ns = steady_now_ns();
    if (command.source_steady_time_ns == 0U || command.source_steady_time_ns > now_ns ||
      now_ns - command.source_steady_time_ns > static_cast<uint64_t>(max_age_ms_ * 1e6)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected stale/future command");
      return;
    }
    if (!enabled_ || !gateway_contract_) {return;}

    // Proposed SAC-ACT-V2. Acknowledgement is not claimed or implemented: it
    // requires matching gateway firmware and HIL validation before enablement.
    std::ostringstream payload;
    payload.precision(10);
    payload << "SAC-ACT-V2,seq=" << command.sequence
      << ",steady_ns=" << command.source_steady_time_ns
      << ",valid_ns=" << (static_cast<uint64_t>(command.validity_duration.sec) * 1000000000ULL +
        command.validity_duration.nanosec)
      << ",mode=" << static_cast<unsigned>(command.mode)
      << ",armed=" << command.armed << ",enable=" << command.enable
      << ",steer_rad=" << command.steering_angle_rad
      << ",steer_rate_radps=" << command.steering_rate_radps
      << ",throttle=" << command.throttle_normalized
      << ",torque_nm=" << command.drive_torque_nm
      << ",brake=" << command.brake_normalized
      << ",speed_mps=" << command.target_speed_mps
      << ",accel_mps2=" << command.target_acceleration_mps2
      << ",flags=" << command.validity_flags;
    const std::string bytes = payload.str();
    const ssize_t sent = sendto(socket_, bytes.data(), bytes.size(), MSG_DONTWAIT,
      reinterpret_cast<const sockaddr *>(&gateway_), sizeof(gateway_));
    if (sent != static_cast<ssize_t>(bytes.size())) {
      RCLCPP_ERROR(get_logger(), "UDP send failed; gateway watchdog must force safe state");
    }
  }
  bool enabled_{false}, gateway_contract_{false};
  bool legacy_cmd_vel_enabled_{false}, legacy_seen_{false};
  double legacy_max_speed_mps_{2.0}, legacy_max_steering_rad_{0.40143};
  std::chrono::steady_clock::time_point legacy_last_received_{};
  std::string address_;
  int port_{4950};
  double max_age_ms_{100.0};
  int socket_{-1};
  sockaddr_in gateway_{};
  rclcpp::Subscription<Command>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr legacy_sub_;
  rclcpp::TimerBase::SharedPtr legacy_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpActuatorTransport>());
  rclcpp::shutdown();
  return 0;
}

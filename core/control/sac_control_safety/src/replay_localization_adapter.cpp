#include <cmath>
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sac_interfaces/msg/pipeline_timing.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ReplayLocalizationAdapter final : public rclcpp::Node
{
public:
  ReplayLocalizationAdapter()
  : Node("replay_localization_adapter")
  {
    if (!declare_parameter("replay_only", false)) {
      throw std::runtime_error("replay_localization_adapter requires replay_only:=true");
    }
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/online/odometry", qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/online/pose", qos);
    timing_pub_ = create_publisher<sac_interfaces::msg::PipelineTiming>(
      "/localization/replay_timing", rclcpp::QoS(1).best_effort());
    initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", qos,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
        tf2::fromMsg(msg.pose.pose, initial_map_pose_);
        initialized_ = true;
        have_origin_ = false;
        RCLCPP_WARN(
          get_logger(),
          "Replay adapter initialized by operator; output remains non-actuating");
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter("recorded_odometry_topic", "/zed/zed_node/odom"),
      rclcpp::SensorDataQoS().keep_last(1),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odom(*msg);});
    RCLCPP_WARN(
      get_logger(), "REPLAY ONLY: waiting for /initialpose; no pose is published before it");
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & input)
  {
    ++received_;
    if (!initialized_ || rclcpp::Time(input.header.stamp).nanoseconds() <= 0) {return;}
    if (last_stamp_.nanoseconds() > 0 && rclcpp::Time(input.header.stamp) <= last_stamp_) {
      ++dropped_;
      return;
    }
    tf2::Transform recorded;
    tf2::fromMsg(input.pose.pose, recorded);
    if (!have_origin_) {
      recorded_origin_inverse_ = recorded.inverse();
      have_origin_ = true;
    }
    const tf2::Transform mapped = initial_map_pose_ * recorded_origin_inverse_ * recorded;
    nav_msgs::msg::Odometry output = input;
    output.header.frame_id = "map";
    output.child_frame_id = "base_link";
    output.pose.pose.position.x = mapped.getOrigin().x();
    output.pose.pose.position.y = mapped.getOrigin().y();
    output.pose.pose.position.z = mapped.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(mapped.getRotation());
    odom_pub_->publish(output);
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header = output.header;
    pose.pose = output.pose;
    pose_pub_->publish(pose);
    last_stamp_ = rclcpp::Time(input.header.stamp);
    ++processed_;

    sac_interfaces::msg::PipelineTiming timing;
    timing.stamp = get_clock()->now();
    timing.component = "replay_localization_adapter";
    timing.sequence = processed_;
    timing.source_age_ms = (get_clock()->now() - last_stamp_).seconds() * 1000.0;
    timing.queue_delay_ms = timing.source_age_ms;
    timing.received_count = received_;
    timing.processed_count = processed_;
    timing.dropped_count = dropped_;
    timing_pub_->publish(timing);
  }

  bool initialized_{false}, have_origin_{false};
  uint64_t received_{0}, processed_{0}, dropped_{0};
  rclcpp::Time last_stamp_{0};
  tf2::Transform initial_map_pose_, recorded_origin_inverse_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sac_interfaces::msg::PipelineTiming>::SharedPtr timing_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReplayLocalizationAdapter>());
  rclcpp::shutdown();
  return 0;
}

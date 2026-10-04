#include <memory>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class OdometryPoseBridge final : public rclcpp::Node
{
public:
  OdometryPoseBridge()
  : Node("online_localization_pose_bridge")
  {
    const auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/online/pose", qos);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/online/odometry", qos,
      [this](const nav_msgs::msg::Odometry & input) {
        geometry_msgs::msg::PoseWithCovarianceStamped output;
        // Preserve the estimator timestamp; using now() would hide delayed data.
        output.header = input.header;
        output.pose = input.pose;
        publisher_->publish(output);
      });
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPoseBridge>());
  rclcpp::shutdown();
  return 0;
}

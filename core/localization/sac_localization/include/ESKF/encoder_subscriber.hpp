#ifndef ENCODER_SUBSCRIBER_HPP_
#define ENCODER_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <mutex>

class EncoderSubscriber : public rclcpp::Node
{
public:
  EncoderSubscriber();
  double getLatestSpeed() const;

private:
  void callback(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

  // raw vs. filtered speed
  mutable std::mutex speed_mutex_;
  double filtered_speed_{0.0};
  double speed_{0.0};

  // LPF smoothing factor (α ∈ (0,1])
  double alpha_{0.1};
};

#endif // ENCODER_SUBSCRIBER_HPP_

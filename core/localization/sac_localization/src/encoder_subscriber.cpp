#include "ESKF/encoder_subscriber.hpp"

EncoderSubscriber::EncoderSubscriber()
  : Node("encoder_subscriber")
{
  // declare a parameter for the LPF coefficient, default to 0.1
  this->declare_parameter<double>("lpf_alpha", 0.05);
  this->get_parameter("lpf_alpha", alpha_);

  subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "encoder_speed", 1,
    std::bind(&EncoderSubscriber::callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "EncoderSubscriber node has started (LPF α=%.3f).",
              alpha_);
}

void EncoderSubscriber::callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  const double raw_speed = msg->data;

  // exponential low-pass filter
  {
    std::lock_guard<std::mutex> lock(speed_mutex_);
    filtered_speed_ = alpha_ * raw_speed + (1.0 - alpha_) * filtered_speed_;
    speed_ = filtered_speed_;
  }

//   RCLCPP_DEBUG(this->get_logger(),
//                "Raw: %.3f  Filtered: %.3f", raw_speed, filtered_speed_);
}

double EncoderSubscriber::getLatestSpeed() const
{
  std::lock_guard<std::mutex> lock(speed_mutex_);
  return speed_;
}

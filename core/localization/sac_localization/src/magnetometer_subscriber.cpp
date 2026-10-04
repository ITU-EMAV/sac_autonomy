#include "ESKF/magnetometer_subscriber.hpp"

MagnetometerSubscriber::MagnetometerSubscriber()
    : Node("magnetometer_subscriber"), latest_mag_(Eigen::Vector3d::Zero()) {
  subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "/zed/zed_node/imu/mag", 10,
      std::bind(&MagnetometerSubscriber::callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "MagnetometerSubscriber has started and is listening to /zed/zed_node/imu/mag");
}

void MagnetometerSubscriber::callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
  Eigen::Vector3d mag;
  mag.x() = msg->magnetic_field.x;
  mag.y() = msg->magnetic_field.y;
  mag.z() = msg->magnetic_field.z;

  {
    std::lock_guard<std::mutex> lock(mag_mutex_);
    latest_mag_ = mag;
  }

  // For debugging:
  // RCLCPP_INFO(this->get_logger(), "Magnetometer: [%.6e, %.6e, %.6e] T", mag.x(), mag.y(), mag.z());
}

Eigen::Vector3d MagnetometerSubscriber::getLatestMag() const {
  std::lock_guard<std::mutex> lock(mag_mutex_);
  return latest_mag_;
}

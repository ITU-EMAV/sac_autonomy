// magnetometer_subscriber.hpp
#ifndef MAGNETOMETER_SUBSCRIBER_HPP
#define MAGNETOMETER_SUBSCRIBER_HPP

#include <mutex>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

class MagnetometerSubscriber : public rclcpp::Node {
public:
  MagnetometerSubscriber();

  // Returns the latest magnetic field in body frame (Tesla)
  Eigen::Vector3d getLatestMag() const;

private:
  void callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr subscription_;
  mutable std::mutex mag_mutex_;
  Eigen::Vector3d latest_mag_;
};

#endif // MAGNETOMETER_SUBSCRIBER_HPP

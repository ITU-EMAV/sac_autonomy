#ifndef GPS_SUBSCRIBER_HPP_
#define GPS_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <Eigen/Dense>
#include <mutex>

// GPSSubscriber class declaration
class GPSSubscriber : public rclcpp::Node {
public:
    // Constructor
    GPSSubscriber();

    // Getter method to retrieve the latest Latitude, Longitude, and Altitude (LLA)
    Eigen::Vector3d getLatestLLA() const;

private:
    // Callback function for the NavSatFix message
    void callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    // ROS 2 subscription for NavSatFix messages
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;

    // Member variable to store the latest LLA values
    Eigen::Vector3d lla_;
    mutable std::mutex lla_mutex_; // Mutex to ensure thread safety
};

#endif // GPS_SUBSCRIBER_HPP_

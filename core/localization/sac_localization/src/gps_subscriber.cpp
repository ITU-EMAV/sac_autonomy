#include "ESKF/gps_subscriber.hpp"

// Constructor implementation
GPSSubscriber::GPSSubscriber()
    : Node("gps_subscriber") // Initialize the ROS 2 node with a name
{
    // Create a subscription to the "navsat/fix" topic
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "navsat/fix", 1,
        std::bind(&GPSSubscriber::callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "GPSSubscriber node has started.");
}

// Callback function implementation
void GPSSubscriber::callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // Create an Eigen vector from the NavSatFix message data
    Eigen::Vector3d new_lla(msg->latitude, msg->longitude, msg->altitude);
    
    // Update the LLA values in a thread-safe manner
    {
        std::lock_guard<std::mutex> lock(lla_mutex_);
        lla_ = new_lla;
    }

    // Uncomment the line below for debugging or logging the updated LLA values
    // RCLCPP_INFO(this->get_logger(), "Updated LLA: [%f, %f, %f]", lla_[0], lla_[1], lla_[2]);
}

// Getter method implementation
Eigen::Vector3d GPSSubscriber::getLatestLLA() const {
    std::lock_guard<std::mutex> lock(lla_mutex_);
    return lla_;
}

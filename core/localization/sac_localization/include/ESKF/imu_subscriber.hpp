#ifndef IMU_SUBSCRIBER_HPP_
#define IMU_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mutex>
#include <memory>
#include <thread>
#include <ESKF/type.hpp>

class IMUSubscriber : public rclcpp::Node
{
public:
    // Constructor
    IMUSubscriber();

    // Getter method to retrieve the latest IMU data
    sensor_msgs::msg::Imu::SharedPtr getLatestIMU();

private:
    // Callback function to process the IMU data
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    // Define placeholders for IMU message
    sensor_msgs::msg::Imu::SharedPtr imu_data_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    mutable std::mutex imu_mutex_; // Mutex to ensure thread safety
};


#endif // IMU_SUBSCRIBER_HPP_
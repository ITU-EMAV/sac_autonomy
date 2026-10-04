#include <ESKF/imu_subscriber.hpp>

IMUSubscriber::IMUSubscriber()
        : Node("imu_subscriber"){
    // Create a subscriber for the IMU topic
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/zed/zed_node/imu/data", 1, std::bind(&IMUSubscriber::imuCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "IMUSubscriber node has started.");
}

    // Getter method to retrieve the latest IMU data
sensor_msgs::msg::Imu::SharedPtr IMUSubscriber::getLatestIMU(){
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_data_;
}

    // Callback function to process the IMU data
void IMUSubscriber::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg){
    // Store the latest IMU data safely using a mutex
    {   
        //RCLCPP_INFO(this->get_logger(), "New Imu data arrived.");

        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_data_ = msg;
    }
}

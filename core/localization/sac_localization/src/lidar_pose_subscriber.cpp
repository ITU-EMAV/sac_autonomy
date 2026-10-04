#include "ESKF/lidar_pose_subscriber.hpp"

LidarPoseSubscriber::LidarPoseSubscriber()
    : Node("lidar_pose_subscriber"),
      has_valid_pose_(false)
{
    // Initialize the pose matrix as identity
    latest_pose_ = Eigen::Matrix4f::Identity();
    
    RCLCPP_INFO(this->get_logger(), "LidarPoseSubscriber initialized");
}

void LidarPoseSubscriber::setTopic(const std::string& topic_name, const std::string& msg_type) {
    // Clear any existing subscriptions
    pose_subscription_.reset();
    pose_with_cov_subscription_.reset();
    odom_subscription_.reset();
    
    // Create the appropriate subscription based on message type
    if (msg_type == "pose") {
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_name, 10, 
            std::bind(&LidarPoseSubscriber::poseCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to PoseStamped on topic: %s", topic_name.c_str());
    } 
    else if (msg_type == "pose_with_covariance") {
        pose_with_cov_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            topic_name, 10, 
            std::bind(&LidarPoseSubscriber::poseWithCovarianceCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to PoseWithCovarianceStamped on topic: %s", topic_name.c_str());
    } 
    else if (msg_type == "odometry") {
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_name, 10, 
            std::bind(&LidarPoseSubscriber::odometryCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to Odometry on topic: %s", topic_name.c_str());
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Unknown message type: %s", msg_type.c_str());
    }
}

void LidarPoseSubscriber::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Extract position
    latest_pose_(0, 3) = msg->pose.position.x;
    latest_pose_(1, 3) = msg->pose.position.y;
    latest_pose_(2, 3) = msg->pose.position.z;
    
    // Extract orientation and convert to rotation matrix
    Eigen::Quaternionf q(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z
    );
    
    // Set the rotation part of the transformation matrix
    latest_pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
    
    // Store the timestamp
    latest_pose_time_ = msg->header.stamp;
    
    // Mark that we have a valid pose
    has_valid_pose_ = true;
}

void LidarPoseSubscriber::poseWithCovarianceCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Extract position
    latest_pose_(0, 3) = msg->pose.pose.position.x;
    latest_pose_(1, 3) = msg->pose.pose.position.y;
    latest_pose_(2, 3) = msg->pose.pose.position.z;
    
    // Extract orientation and convert to rotation matrix
    Eigen::Quaternionf q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    
    // Set the rotation part of the transformation matrix
    latest_pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
    
    // Store the timestamp
    latest_pose_time_ = msg->header.stamp;
    
    // Mark that we have a valid pose
    has_valid_pose_ = true;
}

void LidarPoseSubscriber::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Extract position
    latest_pose_(0, 3) = msg->pose.pose.position.x;
    latest_pose_(1, 3) = msg->pose.pose.position.y;
    latest_pose_(2, 3) = msg->pose.pose.position.z;
    
    // Extract orientation and convert to rotation matrix
    Eigen::Quaternionf q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    
    // Set the rotation part of the transformation matrix
    latest_pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
    
    // Store the timestamp
    latest_pose_time_ = msg->header.stamp;
    
    // Mark that we have a valid pose
    has_valid_pose_ = true;
}

Eigen::Matrix4f LidarPoseSubscriber::getLatestPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_pose_;
}

rclcpp::Time LidarPoseSubscriber::getLatestPoseTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_pose_time_;
}

bool LidarPoseSubscriber::hasValidPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_valid_pose_;
}

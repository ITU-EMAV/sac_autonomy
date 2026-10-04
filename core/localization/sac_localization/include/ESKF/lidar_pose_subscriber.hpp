#ifndef _LIDAR_POSE_SUBSCRIBER_HPP_
#define _LIDAR_POSE_SUBSCRIBER_HPP_

#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>

/**
 * @brief Subscribes to pose estimates from a LiDAR SLAM system
 * 
 * This class subscribes to pose outputs from LiDAR SLAM packages like lidarslam_ros2
 * and makes them available to the localization system. It can handle multiple message
 * types including PoseStamped, PoseWithCovarianceStamped, and Odometry.
 */
class LidarPoseSubscriber : public rclcpp::Node {
public:
    LidarPoseSubscriber();
    ~LidarPoseSubscriber() = default;

    /**
     * @brief Get the latest pose as an Eigen transform matrix
     * @return 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4f getLatestPose() const;
    
    /**
     * @brief Get the time of the latest pose
     * @return time point of the latest pose
     */
    rclcpp::Time getLatestPoseTime() const;
    
    /**
     * @brief Check if a valid pose has been received
     * @return true if a valid pose has been received
     */
    bool hasValidPose() const;
    
    /**
     * @brief Set the topic to subscribe to
     * @param topic_name Name of the topic
     * @param msg_type Type of message ("pose", "pose_with_covariance", or "odometry")
     */
    void setTopic(const std::string& topic_name, const std::string& msg_type);

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void poseWithCovarianceCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_cov_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    
    Eigen::Matrix4f latest_pose_;
    rclcpp::Time latest_pose_time_;
    bool has_valid_pose_;
    
    mutable std::mutex mutex_;
};

#endif // _LIDAR_POSE_SUBSCRIBER_HPP_

#ifndef _EKSF_GNSS_IMU_LOCALIZATION_TYPE_HPP_
#define _EKSF_GNSS_IMU_LOCALIZATION_TYPE_HPP_

#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <mutex>

struct ImuMeasurement {
    using TimePoint = std::chrono::time_point<std::chrono::system_clock>;
    TimePoint timestamp;
    Eigen::Vector3d angularVelocity;
    Eigen::Vector3d acceleration;

    ImuMeasurement(const sensor_msgs::msg::Imu::SharedPtr imuData)
        : timestamp(std::chrono::seconds(imuData->header.stamp.sec) +
                    std::chrono::nanoseconds(imuData->header.stamp.nanosec)),
          angularVelocity(Eigen::Vector3d(imuData->angular_velocity.x,  imuData->angular_velocity.y,
                                          imuData->angular_velocity.z)),
          acceleration(Eigen::Vector3d(imuData->linear_acceleration.x, imuData->linear_acceleration.y,
                                       imuData->linear_acceleration.z)) {}
};

#endif  // _EKSF_GNSS_IMU_LOCALIZATION_TYPE_HPP_
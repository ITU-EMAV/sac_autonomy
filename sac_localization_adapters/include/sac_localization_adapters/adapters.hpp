// The standard sensor adapters of sac_localization (plugins, see adapters.xml).
// Parameters are under sensors.<name>.; the common ones are in TopicAdapter.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "sac_localization/ros/sensor_adapter.hpp"

namespace sac_localization
{

/// sensor_msgs/Imu. use: angular_velocity, linear_acceleration (default), orientation.
///   estimate_biases (true): adds <name>/gyro_bias and <name>/accel_bias to the state
///   angular_velocity_covariance, linear_acceleration_covariance, orientation_covariance:
///     3 variances (or 9 values), used instead of the message's (Gazebo's IMU sends none)
///   angular_velocity_rejection_threshold, linear_acceleration_rejection_threshold,
///   orientation_rejection_threshold: per quantity (default: rejection_threshold). Gyros
///     measure fast real rotations (impacts, slides) that the motion model does not
///     expect: give them a high threshold; accelerometer spikes are better rejected
///   as_input: the readings (in the body frame) drive the imu_driven motion model
class ImuAdapter : public TopicAdapter<sensor_msgs::msg::Imu>
{
public:
  void addStates(StateLayoutBuilder & builder) const override;

protected:
  std::vector<std::string> defaultUse() const override { return {"angular_velocity", "linear_acceleration"}; }
  void configure(const Params & params) override;
  void convert(const sensor_msgs::msg::Imu & message) override;

private:
  bool estimate_biases_ = true;
};

/// sensor_msgs/NavSatFix: the antenna's position in the map (needs the datum), the lever arm
/// from TF. use: horizontal, vertical (default both).
///   covariance: [east, north, up] variances instead of the message's
///   min_status (0 = STATUS_FIX): fixes with a lower status are ignored
class GnssPositionAdapter : public TopicAdapter<sensor_msgs::msg::NavSatFix>
{
protected:
  std::vector<std::string> defaultUse() const override { return {"horizontal", "vertical"}; }
  void configure(const Params & params) override;
  void convert(const sensor_msgs::msg::NavSatFix & message) override;

private:
  int min_status_ = 0;
};

/// sensor_msgs/JointState from the wheels: the rear axle's forward speed (mean of
/// speed_joints x wheel_radius) and the yaw rate (speed tan(mean of steering_joints) /
/// wheel_base). use: speed, yaw_rate (default both).
///   covariance: [speed, yaw rate] variances; rear_axle: [x, y, z] in base_footprint
///   as_input: [speed, steering angle] drive the kinematic_bicycle motion model
class WheelAdapter : public TopicAdapter<sensor_msgs::msg::JointState>
{
protected:
  std::vector<std::string> defaultUse() const override { return {"speed", "yaw_rate"}; }
  void configure(const Params & params) override;
  void convert(const sensor_msgs::msg::JointState & message) override;

private:
  std::vector<std::string> speed_joints_;
  std::vector<std::string> steering_joints_;
  double wheel_radius_ = 0.30;
  double wheel_base_ = 1.873;
  Eigen::Vector3d rear_axle_;
};

/// From a wheel JointState topic: when the speed is below `threshold` the car stands still:
/// body velocity and angular velocity are 0 (this also calibrates gyro biases).
///   speed_joints, wheel_radius, threshold [m/s], covariance: [velocity, angular velocity]
class ZeroVelocityAdapter : public TopicAdapter<sensor_msgs::msg::JointState>
{
protected:
  void configure(const Params & params) override;
  void convert(const sensor_msgs::msg::JointState & message) override;

private:
  std::vector<std::string> speed_joints_;
  double wheel_radius_ = 0.30;
};

/// No topic: a car does not slide sideways or jump. The lateral and vertical velocity of the
/// rear axle are 0, at `rate` Hz.
///   lever_arm: the rear axle in base_footprint; covariance: [lateral, vertical]; enabled
class NonholonomicAdapter : public SensorAdapter
{
public:
  void initialize(
    const AdapterContext & context, const std::string & name, std::shared_ptr<const Params> params) override;
  void start(std::shared_ptr<const StateLayout> layout) override;

private:
  void tick();
  AdapterContext context_;
  std::shared_ptr<const Params> params_;
  Eigen::Vector3d lever_arm_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// nav_msgs/Odometry. use: linear_velocity, angular_velocity (default; in child_frame_id),
/// position, orientation (only when the message's frame is this filter's world frame).
///   linear_velocity_covariance, angular_velocity_covariance, position_covariance,
///   orientation_covariance: 3 variances instead of the message's
class OdometryAdapter : public TopicAdapter<nav_msgs::msg::Odometry>
{
protected:
  std::vector<std::string> defaultUse() const override { return {"linear_velocity", "angular_velocity"}; }
  void convert(const nav_msgs::msg::Odometry & message) override;
};

/// geometry_msgs/TwistWithCovarianceStamped in its header frame. use: linear_velocity,
/// angular_velocity (default).
class TwistAdapter : public TopicAdapter<geometry_msgs::msg::TwistWithCovarianceStamped>
{
protected:
  std::vector<std::string> defaultUse() const override { return {"linear_velocity", "angular_velocity"}; }
  void convert(const geometry_msgs::msg::TwistWithCovarianceStamped & message) override;
};

/// geometry_msgs/PoseWithCovarianceStamped of the sensor frame (`frame`, default the base
/// frame) in this filter's world frame. use: position, orientation (default).
class PoseAdapter : public TopicAdapter<geometry_msgs::msg::PoseWithCovarianceStamped>
{
protected:
  std::vector<std::string> defaultUse() const override { return {"position", "orientation"}; }
  void convert(const geometry_msgs::msg::PoseWithCovarianceStamped & message) override;
};

}  // namespace sac_localization

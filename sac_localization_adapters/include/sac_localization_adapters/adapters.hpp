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

/// The latest reading of an IMU, to cross-check wheel data against (optional).
class ImuWatch
{
public:
  void subscribe(rclcpp::Node * node, const std::string & topic);
  bool active() const { return subscription_ != nullptr; }
  /// A reading no older than `max_age` [s] before `now`
  bool fresh(Stamp now, double max_age = 0.2) const { return has_ && toSeconds(now - stamp_) <= max_age; }
  const Eigen::Vector3d & acceleration() const { return acceleration_; }  // specific force, sensor frame
  const Eigen::Vector3d & angularVelocity() const { return angular_velocity_; }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  bool has_ = false;
  Stamp stamp_ = 0;
  Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_ = Eigen::Vector3d::Zero();
};

/// sensor_msgs/JointState from the wheels: the rear axle's forward speed (mean of
/// speed_joints x wheel_radius) and the yaw rate (speed tan(mean of steering_joints) /
/// wheel_base). use: speed, yaw_rate (default both).
///   covariance: [speed, yaw rate] variances; rear_axle: [x, y, z] in base_footprint
///   as_input: [speed, steering angle] drive the kinematic_bicycle / dynamic_bicycle models
///   also_measure: with as_input, still send the `use` quantities as measurements too
///   slip_check_imu: an IMU topic (x forward); while the wheels' acceleration differs from
///     the IMU's by more than max_slip_acceleration [m/s^2] (locked or spinning wheels) the
///     wheel speed and yaw rate are not sent. The filter cannot reject those itself: the
///     wheel speed drifts away from the car's step by step, each step a small innovation.
class WheelAdapter : public TopicAdapter<sensor_msgs::msg::JointState>
{
protected:
  std::vector<std::string> defaultUse() const override { return {"speed", "yaw_rate"}; }
  void configure(const Params & params) override;
  void convert(const sensor_msgs::msg::JointState & message) override;

private:
  bool slipping(Stamp t, double speed);

  std::vector<std::string> speed_joints_;
  std::vector<std::string> steering_joints_;
  double wheel_radius_ = 0.30;
  double wheel_base_ = 1.873;
  Eigen::Vector3d rear_axle_;
  ImuWatch imu_;
  Stamp last_stamp_ = 0;
  double last_speed_ = 0.0;
  double wheel_acceleration_ = 0.0;  // smoothed
  Stamp slip_until_ = 0;
};

/// From a wheel JointState topic: when the speed is below `threshold` the car stands still:
/// body velocity and angular velocity are 0 (this also calibrates gyro biases).
///   speed_joints, wheel_radius, threshold [m/s], covariance: [velocity, angular velocity]
///   imu_topic: also require the IMU to be still (|specific force| within
///     max_acceleration_deviation of g, |angular velocity| below max_angular_velocity):
///     locked wheels of a sliding car read 0 too
class ZeroVelocityAdapter : public TopicAdapter<sensor_msgs::msg::JointState>
{
protected:
  void configure(const Params & params) override;
  void convert(const sensor_msgs::msg::JointState & message) override;

private:
  std::vector<std::string> speed_joints_;
  double wheel_radius_ = 0.30;
  ImuWatch imu_;
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

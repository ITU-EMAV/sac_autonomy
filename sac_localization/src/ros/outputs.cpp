#include "sac_localization/ros/outputs.hpp"

#include <cmath>
#include <deque>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "sac_localization/core/so3.hpp"
#include "sac_localization_msgs/msg/estimate_error.hpp"
#include "sac_localization_msgs/msg/filter_status.hpp"

namespace sac_localization
{

namespace
{
builtin_interfaces::msg::Time toMsg(Stamp stamp)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(stamp / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
  return t;
}

Eigen::Isometry3d poseOf(const State & x)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = x.position();
  pose.linear() = x.orientation().toRotationMatrix();
  return pose;
}

/// A 6x6 covariance of two 3-blocks of the belief.
Eigen::Matrix<double, 6, 6> covariance6(const Belief & b, const std::string & first, const std::string & second)
{
  const int i = b.state.layout().block(first).tangent_offset;
  const int j = b.state.layout().block(second).tangent_offset;
  Eigen::Matrix<double, 6, 6> c;
  c.block<3, 3>(0, 0) = b.covariance.block<3, 3>(i, i);
  c.block<3, 3>(0, 3) = b.covariance.block<3, 3>(i, j);
  c.block<3, 3>(3, 0) = b.covariance.block<3, 3>(j, i);
  c.block<3, 3>(3, 3) = b.covariance.block<3, 3>(j, j);
  return c;
}

double horizontalStddev(const Belief & b)
{
  const int p = b.state.layout().block(blocks::kPosition).tangent_offset;
  return std::sqrt(std::max(0.0, b.covariance(p, p) + b.covariance(p + 1, p + 1)));
}

double yawStddev(const Belief & b)
{
  const int o = b.state.layout().block(blocks::kOrientation).tangent_offset;
  return std::sqrt(std::max(0.0, b.covariance(o + 2, o + 2)));
}

// ---------------------------------------------------------------- TF
class TfOutput : public Output
{
public:
  TfOutput(rclcpp::Node & node, const AdapterContext & context, std::string odom_frame)
  : node_(node), context_(context), odom_frame_(std::move(odom_frame)), broadcaster_(node) {}

  void publish(const OutputContext & out) override
  {
    Eigen::Isometry3d world_base = poseOf(out.belief.state);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = toMsg(out.belief.stamp);
    t.header.frame_id = context_.world_frame;
    if (context_.world_frame == odom_frame_) {
      t.child_frame_id = context_.base_frame;
      t.transform = tf2::eigenToTransform(world_base).transform;
    } else {
      // map -> odom = (map -> base) (odom -> base)^-1, with the local filter's odom -> base
      Eigen::Isometry3d odom_base;
      try {
        odom_base = tf2::transformToEigen(context_.tf->lookupTransform(
          odom_frame_, context_.base_frame, tf2_ros::fromMsg(t.header.stamp), tf2::durationFromSec(0.0)));
      } catch (const tf2::TransformException &) {
        try {
          odom_base = tf2::transformToEigen(
            context_.tf->lookupTransform(odom_frame_, context_.base_frame, tf2::TimePointZero));
        } catch (const tf2::TransformException & error) {
          RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 5000, "No %s -> %s (is the local filter running?): %s",
            odom_frame_.c_str(), context_.base_frame.c_str(), error.what());
          return;
        }
      }
      t.child_frame_id = odom_frame_;
      t.transform = tf2::eigenToTransform(world_base * odom_base.inverse()).transform;
    }
    broadcaster_.sendTransform(t);
  }

private:
  rclcpp::Node & node_;
  AdapterContext context_;
  std::string odom_frame_;
  tf2_ros::TransformBroadcaster broadcaster_;
};

// ---------------------------------------------------------------- odometry
class OdometryOutput : public Output
{
public:
  OdometryOutput(rclcpp::Node & node, const AdapterContext & context, const std::string & topic)
  : context_(context), publisher_(node.create_publisher<nav_msgs::msg::Odometry>(topic, 10)) {}

  void publish(const OutputContext & out) override
  {
    const State & x = out.belief.state;
    nav_msgs::msg::Odometry m;
    m.header.stamp = toMsg(out.belief.stamp);
    m.header.frame_id = context_.world_frame;
    m.child_frame_id = context_.base_frame;
    m.pose.pose = tf2::toMsg(poseOf(x));
    const auto pose_cov = covariance6(out.belief, blocks::kPosition, blocks::kOrientation);
    const auto twist_cov = covariance6(out.belief, blocks::kLinearVelocity, blocks::kAngularVelocity);
    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        m.pose.covariance[r * 6 + c] = pose_cov(r, c);
        m.twist.covariance[r * 6 + c] = twist_cov(r, c);
      }
    }
    m.twist.twist.linear = tf2::toMsg2(Eigen::Vector3d(x.linearVelocity()));
    const Eigen::Vector3d w = x.angularVelocity();
    m.twist.twist.angular.x = w.x();
    m.twist.twist.angular.y = w.y();
    m.twist.twist.angular.z = w.z();
    publisher_->publish(m);
  }

private:
  AdapterContext context_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

// ---------------------------------------------------------------- fix
class FixOutput : public Output
{
public:
  FixOutput(rclcpp::Node & node, const AdapterContext & context, const std::string & topic)
  : context_(context), publisher_(node.create_publisher<sensor_msgs::msg::NavSatFix>(topic, 10)) {}

  void publish(const OutputContext & out) override
  {
    const Eigen::Vector3d geo = context_.map->toGeodetic(out.belief.state.position());
    sensor_msgs::msg::NavSatFix m;
    m.header.stamp = toMsg(out.belief.stamp);
    m.header.frame_id = context_.base_frame;
    m.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    m.latitude = geo.x();
    m.longitude = geo.y();
    m.altitude = geo.z();
    // Map covariance back to east-north-up
    const int p = out.belief.state.layout().block(blocks::kPosition).tangent_offset;
    const Eigen::Matrix3d enu_from_map = context_.map->mapFromEnu().transpose();
    const Eigen::Matrix3d c = enu_from_map * out.belief.covariance.block<3, 3>(p, p) * enu_from_map.transpose();
    for (int r = 0; r < 3; ++r) {
      for (int k = 0; k < 3; ++k) {
        m.position_covariance[r * 3 + k] = c(r, k);
      }
    }
    m.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    publisher_->publish(m);
  }

private:
  AdapterContext context_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
};

// ---------------------------------------------------------------- status
class StatusOutput : public Output
{
public:
  StatusOutput(rclcpp::Node & node, const std::string & topic)
  : publisher_(node.create_publisher<sac_localization_msgs::msg::FilterStatus>(topic, 10)) {}

  void publish(const OutputContext & out) override
  {
    sac_localization_msgs::msg::FilterStatus m;
    m.header.stamp = toMsg(out.belief.stamp);
    m.estimator = out.estimator;
    m.motion_model = out.motion_model;
    m.initialized = true;
    const int p = out.belief.state.layout().block(blocks::kPosition).tangent_offset;
    for (int i = 0; i < 3; ++i) {
      m.position_stddev.push_back(std::sqrt(std::max(0.0, out.belief.covariance(p + i, p + i))));
    }
    m.yaw_stddev = yawStddev(out.belief);
    for (const auto & adapter : out.adapters) {
      const SensorStatistics & s = adapter->statistics();
      sac_localization_msgs::msg::SensorStatus status;
      status.name = adapter->name();
      status.type = adapter->type();
      status.received = s.received;
      status.accepted = s.accepted;
      status.rejected = s.rejected;
      status.late = s.late;
      status.last_delay = s.last_delay;
      status.last_innovation.assign(s.last_innovation.data(), s.last_innovation.data() + s.last_innovation.size());
      status.last_mahalanobis = s.last_mahalanobis;
      m.sensors.push_back(status);
    }
    publisher_->publish(m);
  }

private:
  rclcpp::Publisher<sac_localization_msgs::msg::FilterStatus>::SharedPtr publisher_;
};

// ---------------------------------------------------------------- error (simulation)
// The ground truth arrives a little after the estimate at the same time, so recent estimates
// are kept and each ground truth pose is compared with the estimate interpolated to its stamp.
class ErrorOutput : public Output
{
public:
  ErrorOutput(rclcpp::Node & node, const std::string & ground_truth, const std::string & topic)
  : publisher_(node.create_publisher<sac_localization_msgs::msg::EstimateError>(topic, 10))
  {
    subscription_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
      ground_truth, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        truth_.push_back(*m);
        while (truth_.size() > 500) {
          truth_.pop_front();
        }
      });
  }

  void publish(const OutputContext & out) override
  {
    Estimate e;
    e.stamp = out.belief.stamp;
    e.position = out.belief.state.position();
    e.yaw = yawOf(out.belief.state.orientation());
    e.horizontal_stddev = horizontalStddev(out.belief);
    e.yaw_stddev = yawStddev(out.belief);
    if (!estimates_.empty() && e.stamp < estimates_.back().stamp) {
      estimates_.clear();  // the clock jumped back
      truth_.clear();
    }
    estimates_.push_back(e);
    while (estimates_.size() > 200) {
      estimates_.pop_front();
    }

    while (!truth_.empty()) {
      const auto & truth = truth_.front();
      const Stamp t = static_cast<Stamp>(truth.header.stamp.sec) * 1000000000LL + truth.header.stamp.nanosec;
      if (t > estimates_.back().stamp) {
        break;  // no estimate after it yet
      }
      for (std::size_t i = 1; i < estimates_.size(); ++i) {
        const Estimate & a = estimates_[i - 1];
        const Estimate & b = estimates_[i];
        if (a.stamp <= t && t <= b.stamp && b.stamp > a.stamp) {
          const double s = static_cast<double>(t - a.stamp) / static_cast<double>(b.stamp - a.stamp);
          Estimate at = s < 0.5 ? a : b;
          at.position = (1.0 - s) * a.position + s * b.position;
          report(truth, at);
          break;
        }
      }
      truth_.pop_front();
    }
  }

private:
  struct Estimate
  {
    Stamp stamp;
    Eigen::Vector3d position;
    double yaw;
    double horizontal_stddev;
    double yaw_stddev;
  };

  void report(const geometry_msgs::msg::PoseStamped & truth, const Estimate & e)
  {
    Eigen::Isometry3d pose;
    tf2::fromMsg(truth.pose, pose);
    const double yaw_true = yawOf(Eigen::Quaterniond(pose.linear()));
    const Eigen::Vector3d d = e.position - pose.translation();
    sac_localization_msgs::msg::EstimateError m;
    m.header.stamp = truth.header.stamp;
    m.horizontal = std::hypot(d.x(), d.y());
    m.vertical = d.z();
    m.along_track = d.x() * std::cos(yaw_true) + d.y() * std::sin(yaw_true);
    m.cross_track = -d.x() * std::sin(yaw_true) + d.y() * std::cos(yaw_true);
    m.yaw = wrapAngle(e.yaw - yaw_true);
    m.horizontal_stddev = e.horizontal_stddev;
    m.yaw_stddev = e.yaw_stddev;
    publisher_->publish(m);
  }

  std::deque<geometry_msgs::msg::PoseStamped> truth_;
  std::deque<Estimate> estimates_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<sac_localization_msgs::msg::EstimateError>::SharedPtr publisher_;
};
}  // namespace

std::vector<std::unique_ptr<Output>> makeOutputs(
  rclcpp::Node & node, const AdapterContext & context, const Params & params)
{
  std::vector<std::unique_ptr<Output>> outputs;
  if (params.getBool("outputs.tf", true)) {
    outputs.push_back(std::make_unique<TfOutput>(node, context, params.getString("odom_frame", "odom")));
  }
  const std::string odometry = params.getString("outputs.odometry", "");
  if (!odometry.empty()) {
    outputs.push_back(std::make_unique<OdometryOutput>(node, context, odometry));
  }
  const std::string fix = params.getString("outputs.fix", "");
  if (!fix.empty()) {
    if (context.map == nullptr) {
      throw std::invalid_argument("outputs.fix needs a datum");
    }
    outputs.push_back(std::make_unique<FixOutput>(node, context, fix));
  }
  const std::string status = params.getString("outputs.status", "");
  if (!status.empty()) {
    outputs.push_back(std::make_unique<StatusOutput>(node, status));
  }
  const std::string ground_truth = params.getString("outputs.ground_truth", "");
  const std::string error = params.getString("outputs.error", "");
  if (!ground_truth.empty() && !error.empty()) {
    outputs.push_back(std::make_unique<ErrorOutput>(node, ground_truth, error));
  }
  return outputs;
}

}  // namespace sac_localization

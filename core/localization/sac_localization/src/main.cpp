#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>
#include <Eigen/Geometry>

// import ros related libs
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#include "ESKF/eskf.hpp"
#include "ESKF/imu_subscriber.hpp"
#include "ESKF/gps_subscriber.hpp"
#include "ESKF/encoder_subscriber.hpp"
#include "ESKF/magnetometer_subscriber.hpp"
#include "ESKF/lidar_pose_subscriber.hpp"
#include "ESKF/math_utils.hpp"


// Enhanced coordinate transformer class to handle LiDAR SLAM to ENU frame transformation
class CoordinateTransformer {
public:
  CoordinateTransformer() : initialized_(false), gps_initialized_(false), 
                           calibration_count_(0), min_calibration_points_(3),
                           use_fixed_transform_(false) {}

  void setParameters(int min_calibration_points, bool use_fixed_transform) {
    min_calibration_points_ = min_calibration_points;
    use_fixed_transform_ = use_fixed_transform;
    
    RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                "Coordinate transformer parameters: min_points=%d, use_fixed_transform=%s",
                min_calibration_points_, use_fixed_transform ? "true" : "false");
  }

  void initializeWithGPS(const Eigen::Vector3d& gps_enu_position, double yaw_rad) {
    // Store initial ENU position from GPS
    gps_enu_origin_ = gps_enu_position;
    initial_yaw_ = yaw_rad;
    
    // Create initial ENU frame rotation matrix from magnetometer yaw
    Eigen::AngleAxisd rotation(yaw_rad, Eigen::Vector3d::UnitZ());
    enu_reference_rotation_ = Eigen::Quaterniond(rotation);
    
    gps_initialized_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                "GPS origin set to ENU position [%.3f, %.3f, %.3f] with yaw %.2f deg",
                gps_enu_origin_.x(), gps_enu_origin_.y(), gps_enu_origin_.z(), 
                yaw_rad * 180.0 / M_PI);
                
    // If using fixed transform, initialize it immediately
    if (use_fixed_transform_) {
      initializeFixedTransform();
    }
  }

  bool addCalibrationPoint(const Eigen::Vector3d& lidar_position, 
                          const Eigen::Quaterniond& lidar_orientation) {
    if (!gps_initialized_) {
      RCLCPP_WARN(rclcpp::get_logger("coordinate_transformer"), 
                  "Cannot add calibration point without GPS initialization!");
      return false;
    }
    
    // Store calibration point
    CalibrationPoint point;
    point.lidar_position = lidar_position;
    point.lidar_orientation = lidar_orientation;
    point.timestamp = std::chrono::steady_clock::now();
    
    calibration_points_.push_back(point);
    calibration_count_++;
    
    RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                "Added calibration point %d/%d: LiDAR pos [%.3f, %.3f, %.3f]",
                calibration_count_, min_calibration_points_, 
                lidar_position.x(), lidar_position.y(), lidar_position.z());
    
    // Calculate transformation if we have enough points
    if (calibration_count_ >= min_calibration_points_) {
      return calculateTransformation();
    }
    
    return false;
  }

  bool isInitialized() const {
    return initialized_;
  }

  bool isGPSInitialized() const {
    return gps_initialized_;
  }

  bool needsMoreCalibrationPoints() const {
    return !use_fixed_transform_ && gps_initialized_ && (calibration_count_ < min_calibration_points_);
  }

  int getCalibrationPointCount() const {
    return calibration_count_;
  }

  Eigen::Vector3d transformLidarToENU(const Eigen::Vector3d& lidar_position) {
    if (!initialized_) {
      auto logger = rclcpp::get_logger("coordinate_transformer");
      
      RCLCPP_INFO(logger, "Coordinate transformer not fully initialized yet!");
      return lidar_position;
    }
    
    // Apply full 3D transformation: 
    // 1. Subtract LiDAR origin offset
    // 2. Apply rotation from LiDAR frame to ENU frame  
    // 3. Add GPS ENU origin
    Eigen::Vector3d relative_position = lidar_position - lidar_origin_offset_;
    Eigen::Vector3d rotated_position = lidar_to_enu_transform_.rotation() * relative_position;
    Eigen::Vector3d translated_position = rotated_position + lidar_to_enu_transform_.translation();
    
    // Apply ENU origin offset (from GPS)
    Eigen::Vector3d final_position = translated_position + gps_enu_origin_;
    
    return final_position;
  }

  Eigen::Quaterniond transformLidarOrientationToENU(const Eigen::Quaterniond& lidar_orientation) {
    if (!initialized_) {
      auto logger = rclcpp::get_logger("coordinate_transformer");
      
      RCLCPP_INFO(logger, "Coordinate transformer not initialized yet!");
      return lidar_orientation;
    }
    
    // Apply rotation transformation to align LiDAR orientation with ENU frame
    // Convert rotation matrix to quaternion first
    Eigen::Quaterniond rotation_quat(lidar_to_enu_transform_.rotation());
    Eigen::Quaterniond enu_orientation = rotation_quat * lidar_orientation;
    
    // Normalize to ensure valid quaternion
    enu_orientation.normalize();
    
    return enu_orientation;
  }

  // Get transformation quality metrics
  double getTransformationQuality() const {
    return transformation_quality_;
  }

  void resetCalibration() {
    calibration_points_.clear();
    calibration_count_ = 0;
    initialized_ = false;
    RCLCPP_WARN(rclcpp::get_logger("coordinate_transformer"), "Calibration reset");
  }

private:
  struct CalibrationPoint {
    Eigen::Vector3d lidar_position;
    Eigen::Quaterniond lidar_orientation;
    std::chrono::steady_clock::time_point timestamp;
  };

  // Initialize a fixed transformation based on common assumptions for robotics systems
  void initializeFixedTransform() {
    RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                "Using fixed transform from LiDAR to ENU frame");
    
    // Define the LiDAR origin to be at (0,0,0) in LiDAR frame
    lidar_origin_offset_ = Eigen::Vector3d::Zero();
    
    // Create transformation based on initial magnetometer yaw
    // Typically, LiDAR frame needs to be rotated to match ENU convention
    lidar_to_enu_transform_.setIdentity();
    
    // Rotation from LiDAR frame to ENU frame
    // Assuming LiDAR has +X forward, +Y left, +Z up (common for many SLAM systems)
    // and ENU has +X east, +Y north, +Z up
    // This means we need to rotate around Z axis by magnetometer yaw
    lidar_to_enu_transform_.linear() = enu_reference_rotation_.toRotationMatrix();
    
    // No translation between frames other than the GPS origin
    // The GPS origin will be added in the transform function
    lidar_to_enu_transform_.translation() = Eigen::Vector3d::Zero();
    
    // Set transformation quality to maximum for fixed transform
    transformation_quality_ = 1.0;
    
    initialized_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                "Fixed coordinate transform initialized with yaw: %.2f°",
                initial_yaw_ * 180.0 / M_PI);
  }

  bool calculateTransformation() {
    if (calibration_points_.size() < min_calibration_points_) {
      return false;
    }
    
    // Use the first calibration point as the origin reference
    const auto& first_point = calibration_points_[0];
    lidar_origin_offset_ = first_point.lidar_position;
    
    // Calculate average orientation from calibration points
    Eigen::Vector4d avg_quat = Eigen::Vector4d::Zero();
    std::vector<double> yaw_angles;
    
    for (const auto& point : calibration_points_) {
      // Extract yaw angle from each calibration point
      tf2::Quaternion q_tf(point.lidar_orientation.x(), point.lidar_orientation.y(),
                           point.lidar_orientation.z(), point.lidar_orientation.w());
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
      yaw_angles.push_back(yaw);
      
      // Accumulate quaternion components for averaging
      avg_quat += Eigen::Vector4d(point.lidar_orientation.w(), point.lidar_orientation.x(),
                                  point.lidar_orientation.y(), point.lidar_orientation.z());
    }
    
    // Normalize averaged quaternion
    avg_quat /= static_cast<double>(calibration_points_.size());
    avg_quat.normalize();
    Eigen::Quaterniond avg_lidar_orientation(avg_quat[0], avg_quat[1], avg_quat[2], avg_quat[3]);
    
    // Calculate average yaw
    double avg_yaw = 0.0;
    for (double yaw : yaw_angles) {
      avg_yaw += yaw;
    }
    avg_yaw /= static_cast<double>(yaw_angles.size());
    
    // Calculate yaw offset between magnetometer ENU frame and LiDAR frame
    double yaw_offset = initial_yaw_ - avg_yaw;
    
    // Normalize yaw offset to [-π, π]
    yaw_offset = std::atan2(std::sin(yaw_offset), std::cos(yaw_offset));
    
    // Create transformation matrix
    // For now, we primarily correct yaw but preserve roll/pitch from LiDAR
    Eigen::AngleAxisd yaw_rotation(yaw_offset, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond yaw_correction(yaw_rotation);
    
    lidar_to_enu_transform_.setIdentity();
    // Set the rotation part using the matrix representation of the quaternion
    lidar_to_enu_transform_.linear() = yaw_correction.toRotationMatrix();
    // Translation offset calculation
    // For multi-point calibration, we can calculate a more robust translation offset
    if (calibration_points_.size() > 3) {
      // Calculate the average position of all calibration points
      Eigen::Vector3d avg_position = Eigen::Vector3d::Zero();
      for (const auto& point : calibration_points_) {
        avg_position += point.lidar_position;
      }
      avg_position /= static_cast<double>(calibration_points_.size());
      
      // Calculate the relative position of the average from the origin point
      Eigen::Vector3d relative_avg = avg_position - lidar_origin_offset_;
      
      // Apply yaw rotation to this relative position
      Eigen::Vector3d rotated_avg = yaw_correction.toRotationMatrix() * relative_avg;
      
      // Calculate the difference between where this point should be in ENU
      // This gives us an additional translation offset to apply
      Eigen::Vector3d translation_offset = -rotated_avg;
      
      // Set the translation component
      lidar_to_enu_transform_.translation() = translation_offset;
      
      RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                  "Calculated translation offset: [%.3f, %.3f, %.3f]",
                  translation_offset.x(), translation_offset.y(), translation_offset.z());
    } else {
      // With fewer points, use a simpler approach
      lidar_to_enu_transform_.translation() = Eigen::Vector3d::Zero();
    }
    
    // Calculate transformation quality based on consistency of calibration points
    calculateTransformationQuality();
    
    // Check if quality meets minimum threshold
    if (transformation_quality_ < 0.5) {
      RCLCPP_WARN(rclcpp::get_logger("coordinate_transformer"), 
                  "Low transformation quality (%.2f). Consider recalibrating or collecting more points.",
                  transformation_quality_);
    }
    
    initialized_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                "Coordinate transformation calculated from %d calibration points:",
                calibration_count_);
    RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                "  LiDAR origin: [%.3f, %.3f, %.3f]",
                lidar_origin_offset_.x(), lidar_origin_offset_.y(), lidar_origin_offset_.z());
    RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                "  Magnetometer yaw: %.2f°, Avg LiDAR yaw: %.2f°, Correction: %.2f°",
                initial_yaw_ * 180.0 / M_PI, avg_yaw * 180.0 / M_PI, yaw_offset * 180.0 / M_PI);
    RCLCPP_INFO(rclcpp::get_logger("coordinate_transformer"), 
                "  Transformation quality: %.3f", transformation_quality_);
    
    return true;
  }

  void calculateTransformationQuality() {
    if (calibration_points_.size() < 2) {
      transformation_quality_ = 0.0;
      return;
    }
    
    // Calculate consistency of yaw angles across calibration points
    std::vector<double> yaw_angles;
    for (const auto& point : calibration_points_) {
      tf2::Quaternion q_tf(point.lidar_orientation.x(), point.lidar_orientation.y(),
                           point.lidar_orientation.z(), point.lidar_orientation.w());
      double roll, pitch, yaw;
      tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
      yaw_angles.push_back(yaw);
    }
    
    // Calculate standard deviation of yaw angles
    double mean_yaw = 0.0;
    for (double yaw : yaw_angles) {
      mean_yaw += yaw;
    }
    mean_yaw /= static_cast<double>(yaw_angles.size());
    
    double variance = 0.0;
    for (double yaw : yaw_angles) {
      double diff = std::atan2(std::sin(yaw - mean_yaw), std::cos(yaw - mean_yaw)); // Angular difference
      variance += diff * diff;
    }
    variance /= static_cast<double>(yaw_angles.size());
    double std_dev = std::sqrt(variance);
    
    // Calculate position consistency
    double position_std_dev = 0.0;
    if (calibration_points_.size() >= 3) {
      // For position quality, we check how well the points fit a line or consistent path
      // This is a simple check for now - could be enhanced with more sophisticated metrics
      Eigen::Vector3d avg_position = Eigen::Vector3d::Zero();
      for (const auto& point : calibration_points_) {
        avg_position += point.lidar_position;
      }
      avg_position /= static_cast<double>(calibration_points_.size());
      
      double pos_variance = 0.0;
      for (const auto& point : calibration_points_) {
        double dist = (point.lidar_position - avg_position).norm();
        pos_variance += dist * dist;
      }
      pos_variance /= static_cast<double>(calibration_points_.size());
      position_std_dev = std::sqrt(pos_variance);
    }
    
    // Quality is inversely related to standard deviation
    // Good quality: std_dev < 0.1 rad (5.7°), Poor quality: std_dev > 0.5 rad (28.6°)
    double yaw_quality = std::max(0.0, 1.0 - (std_dev / 0.5));
    
    // Position quality factor (higher is better)
    double position_quality = 1.0;
    if (position_std_dev > 0.01) {  // Only consider if there's significant variation
      position_quality = std::max(0.0, 1.0 - (position_std_dev / 5.0));  // Normalize to 0-1
    }
    
    // Combined quality metric (weighted average)
    transformation_quality_ = 0.7 * yaw_quality + 0.3 * position_quality;
  }

  // Member variables
  bool initialized_;
  bool gps_initialized_;
  int calibration_count_;
  int min_calibration_points_;
  double transformation_quality_;
  bool use_fixed_transform_;
  
  Eigen::Vector3d gps_enu_origin_;                    // GPS position in ENU frame
  Eigen::Vector3d lidar_origin_offset_;               // First LiDAR pose position
  Eigen::Quaterniond enu_reference_rotation_;         // ENU frame reference rotation
  Eigen::Isometry3d lidar_to_enu_transform_;          // Complete 3D transformation matrix
  double initial_yaw_;                                // Initial magnetometer yaw
  
  std::vector<CalibrationPoint> calibration_points_;  // Calibration data points
};

// Function to publish a NavSatFix message
void publishNavSatFix(
    const rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr& publisher,
    const Eigen::Vector3d& lla, 
    const rclcpp::Clock::SharedPtr& clock) {

    // Create the NavSatFix message
    auto navsat_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();

    // Populate the message
    navsat_msg->header.stamp = clock->now();
    navsat_msg->header.frame_id = "gps";

    navsat_msg->latitude = lla(0);
    navsat_msg->longitude = lla(1);
    navsat_msg->altitude = lla(2);

    // Example covariance (identity matrix for simplicity)
    navsat_msg->position_covariance[0] = 1.0;
    navsat_msg->position_covariance[4] = 1.0;
    navsat_msg->position_covariance[8] = 1.0;
    navsat_msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    // Publish the message
    publisher->publish(*navsat_msg);
}

// Function to publish a NavSatFix message
void publishVector3(
    const rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr& publisher,
    const Eigen::Vector3d& velocity) {

    // Create the NavSatFix message
    auto velocity_msg = std::make_shared<geometry_msgs::msg::Vector3>();

    // Populate the message
    velocity_msg->x = velocity(0);
    velocity_msg->y = velocity(1);
    velocity_msg->z = velocity(2);

    // Publish the message
    publisher->publish(*velocity_msg);
}

// Function to publish a Float32 message
void publishFloat(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr& publisher,
    float value) {

    // Create the Float32 message
    auto float_msg = std::make_shared<std_msgs::msg::Float32>();

    // Populate the message
    float_msg->data = value;

    // Publish the message
    publisher->publish(*float_msg);
}

void publishTransform(
    std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_broadcaster, 
    const std::shared_ptr<rclcpp::Node>& node,
    const Eigen::Vector3d& position, 
    const Eigen::Quaterniond& quaternion
    )

{

    // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped transform;
    
    // Always use node clock for consistency
    // Never future-date localization. Consumers use this acquisition/publication
    // time for fail-closed freshness checks.
    transform.header.stamp = node->get_clock()->now();
    
    // Standard robot localization transform: odom -> base_link
    transform.header.frame_id = "odom";       // Parent frame
    transform.child_frame_id = "base_link";   // Child frame
    
    // Translation
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();

    // Rotation (quaternion)
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();

    
    // Publish the transform
    tf_broadcaster->sendTransform(transform);
}


void publishOdometry(
    const std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>>& odom_publisher,
    const std::shared_ptr<rclcpp::Node>& node,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& velocity,
    const Eigen::Quaterniond& quaternion
) {
    // Create an Odometry message
    nav_msgs::msg::Odometry odom_msg;
    
    // Always use node clock for consistency
    odom_msg.header.stamp = node->get_clock()->now();
    
    odom_msg.header.frame_id = "odom";       // Frame for the odometry data
    odom_msg.child_frame_id = "base_link";  // Frame for the robot

    // Set the position
    odom_msg.pose.pose.position.x = position.x();
    odom_msg.pose.pose.position.y = position.y();
    odom_msg.pose.pose.position.z = 0*position.z();
    odom_msg.pose.pose.orientation.x = quaternion.x();
    odom_msg.pose.pose.orientation.y = quaternion.y();
    odom_msg.pose.pose.orientation.z = quaternion.z();
    odom_msg.pose.pose.orientation.w = quaternion.w();

    // Set the velocity
    odom_msg.twist.twist.linear.x = velocity.x();
    odom_msg.twist.twist.linear.y = velocity.y();
    odom_msg.twist.twist.linear.z = 0*velocity.z();
    odom_msg.twist.twist.angular.x = 0.0;  // Angular velocities can be filled if available
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Convert odom quaternion to tf2 quaternion
    tf2::Quaternion quat(
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w
    );

    // Convert quaternion to Euler angles
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Publish the odometry message
    odom_publisher->publish(odom_msg);
}

int main(int argc, char **argv)
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create a node for parameters
  auto node = rclcpp::Node::make_shared("localization_node");
  
  // Declare required parameters with default values
  node->declare_parameter("localization_mode", "gps");
  node->declare_parameter("lidar.enabled", false);
  node->declare_parameter("gps.enabled", true);
  node->declare_parameter("lidar.pose_topic", "/lidarslam/map_to_odom");
  node->declare_parameter("lidar.pose_msg_type", "pose_with_covariance");
  node->declare_parameter("gps.magnetic_declination_deg", 6.0);
  node->declare_parameter("gps.magnetic_noise_deg", 3.0);
  
  // Now get the parameter values
  std::string localization_mode = node->get_parameter("localization_mode").as_string();
  bool lidar_enabled = node->get_parameter("lidar.enabled").as_bool();
  bool gps_enabled = node->get_parameter("gps.enabled").as_bool();
  
  // Print parameter values for debugging
  RCLCPP_INFO(rclcpp::get_logger("main"), "Localization mode: %s", localization_mode.c_str());
  RCLCPP_INFO(rclcpp::get_logger("main"), "Lidar enabled: %s", lidar_enabled ? "true" : "false");
  RCLCPP_INFO(rclcpp::get_logger("main"), "GPS enabled: %s", gps_enabled ? "true" : "false");
  
  // List all parameters for debugging
  std::vector<std::string> param_names = node->list_parameters({}, 0).names;
  RCLCPP_INFO(rclcpp::get_logger("main"), "Available parameters:");
  for (const auto& name : param_names) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - %s", name.c_str());
  }
  
  // Magnetic declination parameters
  const double declination_deg = node->get_parameter("gps.magnetic_declination_deg").as_double();
  const double declination_rad = declination_deg * M_PI / 180.0;
  const double mag_noise_deg = node->get_parameter("gps.magnetic_noise_deg").as_double();
  const double mag_noise_std_rad = mag_noise_deg * M_PI / 180.0;

  // Create an instance of IMUSubscriber
  auto imu_subscriber = std::make_shared<IMUSubscriber>();
  
  // Create GPS subscriber if enabled
  auto gps_subscriber = std::make_shared<GPSSubscriber>();
  
  // Create encoder subscriber
  auto encoder_subscriber = std::make_shared<EncoderSubscriber>();
  
  // Create magnetometer subscriber
  auto magnetometer_subscriber = std::make_shared<MagnetometerSubscriber>();
  
  // Create LiDAR pose subscriber if enabled
  std::shared_ptr<LidarPoseSubscriber> lidar_pose_subscriber = nullptr;
  if (lidar_enabled || localization_mode == "lidar" || localization_mode == "fusion") {
    lidar_pose_subscriber = std::make_shared<LidarPoseSubscriber>();
    
    // Configure LiDAR pose subscriber
    std::string pose_topic = node->get_parameter("lidar.pose_topic").as_string();
    std::string pose_msg_type = node->get_parameter("lidar.pose_msg_type").as_string();
    lidar_pose_subscriber->setTopic(pose_topic, pose_msg_type);
    
    RCLCPP_WARN(rclcpp::get_logger("main"), 
                "📡 LiDAR SLAM SUBSCRIBER CONFIGURED: Topic='%s', Type='%s'", 
                pose_topic.c_str(), pose_msg_type.c_str());
    RCLCPP_WARN(rclcpp::get_logger("main"), 
                "🔍 LiDAR monitoring enabled for mode: '%s'", 
                localization_mode.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("main"), 
                "❌ LiDAR SLAM SUBSCRIBER NOT CREATED (mode='%s', lidar_enabled=%s)", 
                localization_mode.c_str(), lidar_enabled ? "true" : "false");
  }

  auto publisher_node = rclcpp::Node::make_shared("estimated_states_pub_node");
  auto gps_pos_enu_publisher = publisher_node->create_publisher<geometry_msgs::msg::Vector3>("gps_pos_enu", 10);
  auto estimated_pos_lla_publisher = publisher_node->create_publisher<sensor_msgs::msg::NavSatFix>("estimated_pos_lla", 10);
  auto estimated_pos_enu_publisher = publisher_node->create_publisher<geometry_msgs::msg::Vector3>("estimated_pos_enu", 10);
  auto estimated_vel_enu_publisher = publisher_node->create_publisher<geometry_msgs::msg::Vector3>("estimated_vel_enu", 10);
  auto estimated_orientation_publisher = publisher_node->create_publisher<geometry_msgs::msg::Vector3>("estimated_euler", 10);
  auto estimated_speed_publisher = publisher_node->create_publisher<std_msgs::msg::Float32>("estimated_speed", 10);
  auto odom_publisher = publisher_node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(publisher_node);


  // Spin the node in a separate thread for callback processing
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(imu_subscriber);
  executor->add_node(gps_subscriber);
  executor->add_node(encoder_subscriber);
  executor->add_node(magnetometer_subscriber);
  
  // Add LiDAR pose subscriber if enabled
  if (lidar_pose_subscriber) {
    executor->add_node(lidar_pose_subscriber);
  }
  
  executor->add_node(publisher_node);
  
  // Initialize error state kalman filter class
  auto imuCalibration = YAML::LoadFile(IMU_CALIBRATION_CONFIG_PATH);
  auto eskf = std::make_shared<ErrorStateKalmanFilter>(imuCalibration);
  
  // Create coordinate transformer for LiDAR SLAM to ENU frame transformation
  CoordinateTransformer coordinate_transformer;
  
  // Configure coordinate transformer with parameters from config
  bool use_fixed_transform = node->get_parameter_or("lidar.use_fixed_transform", false);
  int calibration_points = node->get_parameter_or("lidar.calibration_points", 3);
  coordinate_transformer.setParameters(calibration_points, use_fixed_transform);
  
  double min_calibration_quality = node->get_parameter_or("lidar.min_calibration_quality", 0.6);
  bool apply_z_correction = node->get_parameter_or("lidar.apply_z_correction", true);
  
  RCLCPP_INFO(rclcpp::get_logger("main"), 
             "LiDAR to ENU transformation settings: mode=%s, fixed_transform=%s, calibration_points=%d",
             localization_mode.c_str(),
             use_fixed_transform ? "true" : "false",
             calibration_points);

  std::thread executorThread([&executor]() { executor->spin(); });

  auto imu_msg_prev = imu_subscriber->getLatestIMU();
  auto lla_prev = gps_subscriber->getLatestLLA();
  auto speed_msg_prev = encoder_subscriber->getLatestSpeed();
  auto mag_prev = magnetometer_subscriber->getLatestMag();
  
  // Main loop to print the latest IMU data only when new data is received
  auto next_time = std::chrono::steady_clock::now();
  auto interval = std::chrono::milliseconds(1);
  auto next_publish_time = std::chrono::steady_clock::now();
  
  // Get publish frequency from config
  double publish_frequency = node->get_parameter_or("output.publish_frequency", 10.0);
  auto odom_publish_interval = std::chrono::milliseconds(
      static_cast<int>(1000 / publish_frequency));

  // Initialization flags
  bool gps_initialized = false;
  bool mag_initialized = false;
  bool system_initialized = false;
  
  // Track the number of GPS readings received for initialization
  int gps_init_count = 0;
  constexpr int MIN_GPS_INIT_COUNT = 2; // Require 2 valid GPS readings for initialization
  
  // Initial position from GPS
  Eigen::Vector3d initial_lla(0, 0, 0);
  
  RCLCPP_INFO(rclcpp::get_logger("main"), "Waiting for GPS and magnetometer data to initialize...");
  
  while (rclcpp::ok())
  {   
    // Step 1: Initialize with GPS position if not already done
    if (!gps_initialized) {
      auto lla = gps_subscriber->getLatestLLA();
      
      // Check if we have valid GPS data (non-zero)
      if (lla.norm() > 0.0 && lla != lla_prev) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Received GPS data: %.6f, %.6f, %.6f", 
                   lla(0), lla(1), lla(2));
        
        // Accumulate GPS position for initialization
        initial_lla += lla;
        gps_init_count++;
        lla_prev = lla;
        
        // Once we have enough GPS readings, calculate average position and initialize
        if (gps_init_count >= MIN_GPS_INIT_COUNT) {
          initial_lla /= gps_init_count;
          
          // Initialize the ENU coordinate system
          Eigen::Vector3d gps_enu_position = eskf->llaToEnu(initial_lla); // This initializes the coordinate system
          eskf->updateWithGnss(initial_lla); // This updates the ESKF with the GPS position
          
          RCLCPP_INFO(rclcpp::get_logger("main"), "GPS initialization complete with position: %.6f, %.6f, %.6f", 
                     initial_lla(0), initial_lla(1), initial_lla(2));
          RCLCPP_INFO(rclcpp::get_logger("main"), "GPS ENU position: %.2f, %.2f, %.2f", 
                     gps_enu_position.x(), gps_enu_position.y(), gps_enu_position.z());
          
          gps_initialized = true;
        }
      }
      
      if (!gps_initialized) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
    }

    // Step 2: Initialize with magnetometer orientation if not already done
    if (gps_initialized && !mag_initialized) {
      Eigen::Vector3d mag = magnetometer_subscriber->getLatestMag();
      
      // Check if we have valid magnetometer data (non-zero)
      if (mag.norm() > 0.0 && mag != mag_prev) {
        // Calculate yaw from magnetometer with declination correction
        double yaw_meas = std::atan2(mag.x(), mag.y()) - declination_rad;
        yaw_meas = std::atan2(std::sin(yaw_meas), std::cos(yaw_meas));
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "Magnetometer initialization with yaw: %.2f degrees", 
                   yaw_meas * 180.0 / M_PI);
        
        // Update the ESKF with the magnetometer yaw
        eskf->updateWithMagYaw(yaw_meas);
        mag_prev = mag;
        
        // Initialize coordinate transformer with GPS position and magnetometer yaw
        Eigen::Vector3d gps_enu_position = eskf->llaToEnu(initial_lla);
        coordinate_transformer.initializeWithGPS(gps_enu_position, yaw_meas);
        
        mag_initialized = true;
      }
      
      if (!mag_initialized) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
    }
    
    // If both GPS and magnetometer are initialized, we're ready to go
    if (gps_initialized && mag_initialized && !system_initialized) {
      RCLCPP_INFO(rclcpp::get_logger("main"), 
                 "System initialization complete! Localization mode: %s", 
                 localization_mode.c_str());
      system_initialized = true;
    }
    
    // Only proceed with the main loop if initialization is complete
    if (!system_initialized) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    auto imu_msg = imu_subscriber->getLatestIMU();

    // Check if new data has arrived
    if (imu_msg && imu_msg != imu_msg_prev)
    {
      auto imuMeasurement = std::make_shared<ImuMeasurement>(imu_msg);
      //std::cout << "imuMeasurement : " << imuMeasurement->acceleration(0) << imuMeasurement->acceleration(1) << imuMeasurement->acceleration(2) << std::endl;
      
      //std::cout << "filteredPosition : " << filteredPosition(0) << filteredPosition(1) << filteredPosition(2) << std::endl;
      eskf->predictWithImu(std::move(imuMeasurement));
      
      // Update the previous message
      imu_msg_prev = imu_msg;
    }

    auto speed_msg = encoder_subscriber->getLatestSpeed();

    if (speed_msg != speed_msg_prev){
      // eskf->updateWithWheelEncoder(speed_msg);

      speed_msg_prev = speed_msg;

    }

    if (std::chrono::steady_clock::now() > next_publish_time){
      // std::cout << "latestLLA : " << lla(0) << lla(1) << lla(2) << std::endl;
      // auto gnssPosition = eskf->llaToEnu(lla);
      auto filteredPosition = eskf->getPosition();
      auto filteredVelocity = eskf->getVelocity();
      double filteredSpeed = filteredVelocity.norm();
      auto filteredPosLLA = eskf->enuToLla(filteredPosition);
      auto filteredQuat = eskf->getQuaternion(); // in imu frame

      // Convert to tf2::Quaternion
    tf2::Quaternion q_tf(
        filteredQuat.x(),
        filteredQuat.y(),
        filteredQuat.z(),
        filteredQuat.w());

    // Compute roll, pitch, yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

    // Optionally convert to degrees
    double roll_deg  = roll  * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;
    double yaw_deg   = yaw   * 180.0 / M_PI;

    // // Print to console
    // std::cout << "[RPY deg] roll="
    //           << roll_deg << "  pitch="
    //           << pitch_deg << "  yaw="
    //           << yaw_deg << std::endl;

      // Create a quaternion representing a rotation around the Z-axis (yaw)
      // double yaw_offset = - M_PI;  // 180 degrees
      // Eigen::AngleAxisd yaw_rotation(yaw_offset, Eigen::Vector3d::UnitZ());
      // Eigen::Quaterniond q_yaw(yaw_rotation);
      // // Apply the yaw rotation
      // filteredQuat = q_yaw * filteredQuat;

      auto filteredEulerAngles = MathUtils::quat2Euler(filteredQuat); 

      // Calculate velocity heading
      double velocityYaw = std::atan2(filteredVelocity.y(), filteredVelocity.x());
      
      // Get latest magnetometer reading for debug output
      Eigen::Vector3d current_mag = magnetometer_subscriber->getLatestMag();
      if (current_mag.norm() > 0.0) {
        double yaw_meas = std::atan2(current_mag.x(), current_mag.y()) - declination_rad;
        yaw_meas = std::atan2(std::sin(yaw_meas), std::cos(yaw_meas));
        // std::cout << "Magnetometer Yaw:" << yaw_meas * 180 / M_PI << std::endl;
      }
     
      publishNavSatFix(estimated_pos_lla_publisher, filteredPosLLA, publisher_node->get_clock());
      publishVector3(estimated_pos_enu_publisher, filteredPosition);
      publishVector3(estimated_vel_enu_publisher, filteredVelocity);
      publishVector3(estimated_orientation_publisher, filteredEulerAngles);
      publishFloat(estimated_speed_publisher, filteredSpeed);
      publishOdometry(odom_publisher, publisher_node, filteredPosition, filteredVelocity, filteredQuat);
      
      // Publish transform if enabled in config and not in pure LiDAR mode
      // In LiDAR mode, let LiDAR SLAM handle the map->odom transform
      bool publish_tf = node->get_parameter_or("output.publish_tf", false);
      if (publish_tf && localization_mode != "lidar") {
        publishTransform(tf_broadcaster, publisher_node, filteredPosition, filteredQuat);
      } else if (localization_mode == "lidar") {
        RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 5000,
                             "Transform publishing disabled in LiDAR mode to avoid conflicts with LiDAR SLAM");
      }
      
      next_publish_time += odom_publish_interval;
      
      // Periodic status report for LiDAR usage monitoring
      static auto last_status_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_status_time).count() >= 10) {
        std::string coord_status;
        if (coordinate_transformer.isInitialized()) {
          coord_status = "READY(Q:" + std::to_string(coordinate_transformer.getTransformationQuality()).substr(0,4) + ")";
        } else if (coordinate_transformer.needsMoreCalibrationPoints()) {
          coord_status = "CALIBRATING(" + std::to_string(coordinate_transformer.getCalibrationPointCount()) + "/3)";
        } else {
          coord_status = "WAITING";
        }
        
        RCLCPP_INFO(rclcpp::get_logger("main"), 
                   "STATUS: Mode='%s' | GPS=%s | LiDAR_sub=%s | LiDAR_data=%s | Coord_trans=%s", 
                   localization_mode.c_str(),
                   gps_enabled ? "ON" : "OFF",
                   lidar_pose_subscriber ? "CREATED" : "NULL",
                   (lidar_pose_subscriber && lidar_pose_subscriber->hasValidPose()) ? "VALID" : "NONE",
                   coord_status.c_str());
        last_status_time = now;
      }
    }
  



    auto lla = gps_subscriber->getLatestLLA();
    
    bool gps_enabled = node->get_parameter_or("gps.enabled", false);
    if (gps_enabled && lla != lla_prev) {
      // Update ESKF with GNSS data if GPS is enabled
      eskf->updateWithGnss(lla);
      publishVector3(gps_pos_enu_publisher, eskf->llaToEnu(lla));
      lla_prev = lla;
    }

    // Use LiDAR data for localization if enabled and available
    static bool lidar_warned_no_data = false;
    static bool lidar_first_data_logged = false;
    
    if (lidar_pose_subscriber) {
      if (lidar_pose_subscriber->hasValidPose()) {
        if (!lidar_first_data_logged) {
          RCLCPP_WARN(rclcpp::get_logger("main"), "FIRST LIDAR SLAM DATA RECEIVED!");
          lidar_first_data_logged = true;
        }
        lidar_warned_no_data = false; // Reset warning flag
        
        if (localization_mode == "lidar" || localization_mode == "fusion") {
        // Get the latest LiDAR pose with timestamp
        Eigen::Matrix4f lidar_pose = lidar_pose_subscriber->getLatestPose();
        rclcpp::Time lidar_time = lidar_pose_subscriber->getLatestPoseTime();
        
        // Extract position from the transformation matrix
        Eigen::Vector3d lidar_position(
            lidar_pose(0, 3),
            lidar_pose(1, 3),
            lidar_pose(2, 3)
        );
        
        // Extract rotation from the transformation matrix
        Eigen::Matrix3f rotation_matrix = lidar_pose.block<3, 3>(0, 0);
        Eigen::Quaternionf quat_f(rotation_matrix);
        Eigen::Quaterniond lidar_orientation(quat_f.w(), quat_f.x(), quat_f.y(), quat_f.z());
        
        // Extract yaw from LiDAR orientation for debugging
        tf2::Quaternion q_debug_tf(
            lidar_orientation.x(), lidar_orientation.y(), 
            lidar_orientation.z(), lidar_orientation.w());
        double debug_roll, debug_pitch, debug_yaw;
        tf2::Matrix3x3(q_debug_tf).getRPY(debug_roll, debug_pitch, debug_yaw);
        
        // Log that we're using LiDAR data
        static int lidar_usage_count = 0;
        lidar_usage_count++;
        
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 1000,
                            "USING LIDAR SLAM DATA #%d - Raw position: [%.3f, %.3f, %.3f], yaw: %.1f°", 
                            lidar_usage_count,
                            lidar_position.x(), lidar_position.y(), lidar_position.z(),
                            debug_yaw * 180.0 / M_PI);
        
        // Add calibration points for coordinate transformer if needed
        if (coordinate_transformer.isGPSInitialized() && !coordinate_transformer.isInitialized()) {
          bool calibration_complete = coordinate_transformer.addCalibrationPoint(lidar_position, lidar_orientation);
          
          if (coordinate_transformer.needsMoreCalibrationPoints()) {
            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 2000,
                                "📍 Collecting calibration point %d/%d for coordinate transformation...",
                                coordinate_transformer.getCalibrationPointCount(), calibration_points);
          }
          
          if (calibration_complete) {
            RCLCPP_WARN(rclcpp::get_logger("main"), 
                       "✅ COORDINATE TRANSFORMER CALIBRATED with %d points! Quality: %.3f",
                       coordinate_transformer.getCalibrationPointCount(),
                       coordinate_transformer.getTransformationQuality());
            
            // Verify if quality meets minimum threshold
            if (coordinate_transformer.getTransformationQuality() < min_calibration_quality) {
              RCLCPP_ERROR(rclcpp::get_logger("main"), 
                          "⚠️ LOW QUALITY CALIBRATION (%.2f < %.2f). Resetting to collect better data.",
                          coordinate_transformer.getTransformationQuality(), min_calibration_quality);
              coordinate_transformer.resetCalibration();
            }
          }
        }
        
        // Transform LiDAR pose to ENU frame if coordinate transformer is ready
        Eigen::Vector3d enu_position = lidar_position;
        Eigen::Quaterniond enu_orientation = lidar_orientation;
        bool transformation_applied = false;
        
        if (coordinate_transformer.isInitialized()) {
          // Apply transformation
          enu_position = coordinate_transformer.transformLidarToENU(lidar_position);
          enu_orientation = coordinate_transformer.transformLidarOrientationToENU(lidar_orientation);
          transformation_applied = true;
          
          // Apply Z-axis (height) correction if enabled
          if (apply_z_correction) {
            // Get current GPS height if available
            auto lla = gps_subscriber->getLatestLLA();
            if (lla.norm() > 0.0) {
              Eigen::Vector3d gps_enu = eskf->llaToEnu(lla);
              // Use GPS height but keep XY from LiDAR
              enu_position.z() = gps_enu.z();
              RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 5000,
                          "Applied GPS height correction: %.2f", gps_enu.z());
            }
          }
          
          // Extract transformed yaw for debugging
          tf2::Quaternion q_enu_tf(
              enu_orientation.x(), enu_orientation.y(), 
              enu_orientation.z(), enu_orientation.w());
          double enu_roll, enu_pitch, enu_yaw;
          tf2::Matrix3x3(q_enu_tf).getRPY(enu_roll, enu_pitch, enu_yaw);
          
          RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 1000,
                      "Transformed LiDAR position [%.3f, %.3f, %.3f] -> ENU [%.3f, %.3f, %.3f]",
                      lidar_position.x(), lidar_position.y(), lidar_position.z(),
                      enu_position.x(), enu_position.y(), enu_position.z());
          
          RCLCPP_INFO_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 3000,
                      "🧭 COORDINATE TRANSFORM: LiDAR yaw %.1f° -> ENU yaw %.1f° | Quality: %.3f | Δpos: [%.2f,%.2f,%.2f]",
                      debug_yaw * 180.0 / M_PI, enu_yaw * 180.0 / M_PI,
                      coordinate_transformer.getTransformationQuality(),
                      enu_position.x() - lidar_position.x(),
                      enu_position.y() - lidar_position.y(),
                      enu_position.z() - lidar_position.z());
        } else if (coordinate_transformer.isGPSInitialized() && !coordinate_transformer.isInitialized()) {
          RCLCPP_WARN_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 3000,
                               "⏳ Collecting calibration data (%d/%d points) for coordinate transformation...",
                               coordinate_transformer.getCalibrationPointCount(), calibration_points);
        } else {
          RCLCPP_WARN_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 5000,
                               "⚠️  Coordinate transformer not ready - using raw LiDAR poses");
        }
        
        // In fusion mode, we use both LiDAR and GPS for position updates
        if (localization_mode == "fusion") {
          // Get the current position estimate
          Eigen::Vector3d current_position = eskf->getPosition();
          
          // Create a weighted fusion of LiDAR and current position
          double fusion_weight = node->get_parameter_or("lidar.fusion_weight", 0.9);
          Eigen::Vector3d fused_position = fusion_weight * enu_position + (1 - fusion_weight) * current_position;
          
          // Convert to LLA and update ESKF
          Eigen::Vector3d fused_lla = eskf->enuToLla(fused_position);
          eskf->updateWithGnss(fused_lla);
          
          RCLCPP_INFO_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 2000,
                              "FUSION MODE: Current=[%.3f,%.3f,%.3f] + LiDAR=[%.3f,%.3f,%.3f] (weight=%.2f) -> Fused=[%.3f,%.3f,%.3f]%s", 
                              current_position.x(), current_position.y(), current_position.z(),
                              enu_position.x(), enu_position.y(), enu_position.z(),
                              fusion_weight,
                              fused_position.x(), fused_position.y(), fused_position.z(),
                              transformation_applied ? " [TRANSFORMED]" : " [RAW]");
        } 
        else if (localization_mode == "lidar") {
          // In pure LiDAR mode, we use transformed LiDAR data for position updates
          Eigen::Vector3d lidar_lla = eskf->enuToLla(enu_position);
          eskf->updateWithGnss(lidar_lla);
          
          RCLCPP_INFO_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 2000,
                              "LIDAR MODE: Using position [%.3f, %.3f, %.3f]%s for ESKF update", 
                              enu_position.x(), enu_position.y(), enu_position.z(),
                              transformation_applied ? " [TRANSFORMED from LiDAR frame]" : " [RAW LiDAR frame]");
        }
        }
      } else {
        // LiDAR subscriber exists but no valid pose data
        if ((localization_mode == "lidar" || localization_mode == "fusion") && !lidar_warned_no_data) {
          RCLCPP_WARN(rclcpp::get_logger("main"), 
                     "⏳ WAITING for LiDAR SLAM data on topic '%s' (type: '%s')...", 
                     node->get_parameter("lidar.pose_topic").as_string().c_str(),
                     node->get_parameter("lidar.pose_msg_type").as_string().c_str());
          lidar_warned_no_data = true;
        }
      }
    } else if (localization_mode == "lidar" || localization_mode == "fusion") {
      // LiDAR mode is expected but no subscriber was created
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("main"), *node->get_clock(), 5000,
                            "❌ CRITICAL: LiDAR mode ('%s') requested but no LiDAR subscriber created!", 
                            localization_mode.c_str());
    }
  }

  // Shutdown ROS 2
  executor->cancel();
  if (executorThread.joinable()) {
    executorThread.join();
  }
  rclcpp::shutdown();
  return 0;
}

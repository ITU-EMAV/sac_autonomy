// src/lidar_filter.cpp

#include "lidar_occupancy_grid/lidar_filter.hpp"

LidarFilter::LidarFilter(const rclcpp::NodeOptions & options)
: Node("lidar_filter", options)
{
  // Declare & read parameters
  this->declare_parameter("min_range", 0.5);
  this->declare_parameter("max_range", 100.0);
  this->declare_parameter("crop_min_x", -10.0);
  this->declare_parameter("crop_max_x", 10.0);
  this->declare_parameter("crop_min_y", -10.0);
  this->declare_parameter("crop_max_y", 10.0);
  this->declare_parameter("crop_min_z", -2.0);
  this->declare_parameter("crop_max_z", 2.0);
  this->declare_parameter("frame_id", "ego_vehicle");
  this->declare_parameter("input_topic", "input_cloud");
  this->declare_parameter("output_topic", "filtered_point_cloud");

  this->get_parameter("min_range",   min_range_);
  this->get_parameter("max_range",   max_range_);
  this->get_parameter("crop_min_x",  crop_min_x_);
  this->get_parameter("crop_max_x",  crop_max_x_);
  this->get_parameter("crop_min_y",  crop_min_y_);
  this->get_parameter("crop_max_y",  crop_max_y_);
  this->get_parameter("crop_min_z",  crop_min_z_);
  this->get_parameter("crop_max_z",  crop_max_z_);
  this->get_parameter("frame_id",    frame_id_);
  this->get_parameter("input_topic", input_topic_);
  this->get_parameter("output_topic", output_topic_);

  RCLCPP_INFO(this->get_logger(), "LidarFilter initialized with parameters: \n"
              "  min_range: %.2f, max_range: %.2f\n"
              "  crop_min_x: %.2f, crop_max_x: %.2f\n"
              "  crop_min_y: %.2f, crop_max_y: %.2f\n"
              "  crop_min_z: %.2f, crop_max_z: %.2f\n"
              "  frame_id: %s\n"
              "  input_topic: %s\n"
              "  output_topic: %s",
              min_range_, max_range_,
              crop_min_x_, crop_max_x_,
              crop_min_y_, crop_max_y_,
              crop_min_z_, crop_max_z_,
              frame_id_.c_str(),
              input_topic_.c_str(),
              output_topic_.c_str());

  // Create subscription & publisher
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::QoS(10),
    std::bind(&LidarFilter::pointCloudCallback, this, std::placeholders::_1)
  );

  filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_, rclcpp::QoS(10)
  );
}

void LidarFilter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto input_cloud = std::make_shared<pcl::PointCloud<PointType>>();
  pcl::fromROSMsg(*msg, *input_cloud);

  auto filtered_cloud = std::make_shared<pcl::PointCloud<PointType>>();
  filterPointCloud(input_cloud, filtered_cloud);

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*filtered_cloud, output_msg);
  output_msg.header = msg->header;
  output_msg.header.frame_id = frame_id_;

  filtered_cloud_pub_->publish(output_msg);
  // RCLCPP_INFO(this->get_logger(), "Filtered point cloud published with %zu points", filtered_cloud->size()); // Debugging line
}

void LidarFilter::filterPointCloud(
    const pcl::PointCloud<PointType>::Ptr & input_cloud,
    pcl::PointCloud<PointType>::Ptr & output_cloud)
{
  // Pass-through on X
  pass_through_filter_.setInputCloud(input_cloud);
  pass_through_filter_.setFilterFieldName("x");
  pass_through_filter_.setFilterLimits(min_range_, max_range_);
  pass_through_filter_.filter(*output_cloud);

  // Crop box
  crop_box_filter_.setInputCloud(output_cloud);
  crop_box_filter_.setMin(Eigen::Vector4f(crop_min_x_, crop_min_y_, crop_min_z_, 1.0f));
  crop_box_filter_.setMax(Eigen::Vector4f(crop_max_x_, crop_max_y_, crop_max_z_, 1.0f));
  crop_box_filter_.filter(*output_cloud);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFilter>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

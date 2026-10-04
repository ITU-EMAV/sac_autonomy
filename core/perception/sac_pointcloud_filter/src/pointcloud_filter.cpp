#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudFilter final : public rclcpp::Node
{
public:
  PointCloudFilter() : Node("pointcloud_filter")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/velodyne_points");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/sensing/points_filtered");
    min_range_ = declare_parameter<double>("min_range", 1.0);
    max_range_ = declare_parameter<double>("max_range", 120.0);
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.0);

    if (min_range_ < 0.0 || max_range_ < 0.0 ||
      (max_range_ > 0.0 && min_range_ >= max_range_) || voxel_leaf_size_ < 0.0)
    {
      throw std::invalid_argument("Invalid range or voxel_leaf_size parameter");
    }

    const auto input_qos = rclcpp::SensorDataQoS().keep_last(5);
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, input_qos,
      std::bind(&PointCloudFilter::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "C++ pointcloud_filter: %s -> %s", input_topic.c_str(),
      output_topic.c_str());
    RCLCPP_INFO(get_logger(), "  range [%.1f, %.1f] m, voxel=%.2f m", min_range_, max_range_,
      voxel_leaf_size_);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *input);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ranged(new pcl::PointCloud<pcl::PointXYZI>);
    ranged->header = input->header;
    ranged->points.reserve(input->points.size());

    const float min_squared = static_cast<float>(min_range_ * min_range_);
    const float max_squared = static_cast<float>(max_range_ * max_range_);
    for (const auto & point : input->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      const float squared_range = point.x * point.x + point.y * point.y + point.z * point.z;
      if ((min_range_ > 0.0 && squared_range < min_squared) ||
        (max_range_ > 0.0 && squared_range > max_squared))
      {
        continue;
      }
      ranged->points.push_back(point);
    }
    ranged->width = static_cast<std::uint32_t>(ranged->points.size());
    ranged->height = 1;
    ranged->is_dense = true;

    pcl::PointCloud<pcl::PointXYZI>::Ptr output = ranged;
    if (voxel_leaf_size_ > 0.0 && !ranged->empty()) {
      output = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
      const float leaf = static_cast<float>(voxel_leaf_size_);
      voxel_filter.setLeafSize(leaf, leaf, leaf);
      voxel_filter.setInputCloud(ranged);
      voxel_filter.filter(*output);
    }

    sensor_msgs::msg::PointCloud2 result;
    pcl::toROSMsg(*output, result);
    result.header = msg->header;
    publisher_->publish(result);
  }

  double min_range_{};
  double max_range_{};
  double voxel_leaf_size_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilter>());
  rclcpp::shutdown();
  return 0;
}

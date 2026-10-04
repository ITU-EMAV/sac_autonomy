#ifndef LIDAR_FILTER_HPP
#define LIDAR_FILTER_HPP
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// Define the point type used in the point cloud
using PointType = pcl::PointXYZI;

class LidarFilter : public rclcpp::Node
{
public:
    LidarFilter(const rclcpp::NodeOptions & options);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
private:
    void filterPointCloud(const pcl::PointCloud<PointType>::Ptr &input_cloud,
                          pcl::PointCloud<PointType>::Ptr &output_cloud);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    pcl::PassThrough<PointType> pass_through_filter_;
    pcl::CropBox<PointType> crop_box_filter_;
    double min_range_;
    double max_range_;
    double crop_min_x_;
    double crop_max_x_;
    double crop_min_y_;
    double crop_max_y_;
    double crop_min_z_;
    double crop_max_z_;
    std::string frame_id_;
    std::string input_topic_;
    std::string output_topic_;
};

#endif // LIDAR_FILTER_HPP
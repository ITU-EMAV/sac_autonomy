#ifndef PC_TO_OG_NODE_PC_TO_OG_HPP
#define PC_TO_OG_NODE_PC_TO_OG_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>

class PointCloudToOccupancyGrid : public rclcpp::Node
{
public:
    PointCloudToOccupancyGrid();

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // ROS publisher and subscription.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    // Occupancy grid message.
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg_;

    // Grid parameters.
    double resolution_;
    double grid_width_m_;
    double grid_height_m_;
    int grid_width_cells_;
    int grid_height_cells_;
    double MIN_Z;

    // Dilation parameters.
    bool expand_occupied_cells_;
    cv::Mat dilation_kernel_;
};

#endif // PC_TO_OG_NODE_PC_TO_OG_HPP

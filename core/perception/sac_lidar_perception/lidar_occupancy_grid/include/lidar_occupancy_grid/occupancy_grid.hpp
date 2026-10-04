#ifndef OCCUPANCY_GRID_HPP
#define OCCUPANCY_GRID_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cluster_msgs/msg/cluster_points_array.hpp>
#include <vector>

class BottomClustersToOccupancyGridNode : public rclcpp::Node
{
public:
    BottomClustersToOccupancyGridNode();

private:
    void clustersCallback(const cluster_msgs::msg::ClusterPointsArray::SharedPtr msg);
    void updateLogOdds(int idx, bool occupied);

    // Parameters
    double grid_resolution_;
    int grid_size_;
    float MinProbability;
    float MaxProbability;

    // Occupancy grid internal data
    std::vector<float> grid_data_;
    std::vector<int8_t> grid_data_global;
    std::vector<float> log_odds_grid_;
    float l_occ;
    float l_free;

    int start_x;
    int start_y;
    std::string frame_id;

    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Subscription<cluster_msgs::msg::ClusterPointsArray>::SharedPtr bottom_clusters_sub_;
    nav_msgs::msg::OccupancyGrid current_grid_;
};

#endif // OCCUPANCY_GRID_HPP
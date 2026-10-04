#include "lidar_occupancy_grid/occupancy_grid.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>

BottomClustersToOccupancyGridNode::BottomClustersToOccupancyGridNode()
: Node("bottom_clusters_to_occupancy_grid")
{
    declare_parameter<double>("grid_resolution", 0.1);
    declare_parameter<int>("grid_size", 100);
    declare_parameter<float>("MinProbability", 0.2f);
    declare_parameter<float>("MaxProbability", 0.8f);
    declare_parameter<std::string>("frame_id", "velodyne");
    get_parameter("grid_resolution", grid_resolution_);
    get_parameter("grid_size", grid_size_);
    get_parameter("MinProbability", MinProbability);
    get_parameter("MaxProbability", MaxProbability);
    get_parameter("frame_id", frame_id);

    l_free = std::log(MinProbability / (1.0f - MinProbability));
    l_occ  = std::log(MaxProbability / (1.0f - MaxProbability));
    constexpr float l0 = 0.0f;

    grid_data_.assign(grid_size_ * grid_size_, 0.5f);
    log_odds_grid_.assign(grid_size_ * grid_size_, l0);

    current_grid_.info.resolution = grid_resolution_;
    current_grid_.info.width      = grid_size_;
    current_grid_.info.height     = grid_size_;
    current_grid_.info.origin.position.x = -((grid_size_ * grid_resolution_) / 2.0);
    current_grid_.info.origin.position.y = -((grid_size_ * grid_resolution_) / 2.0);

    // This QoS profile ensures the last published map is saved and sent to new subscribers
    rclcpp::QoS latching_qos(rclcpp::KeepLast(1));
    latching_qos.transient_local(); // This makes it "latching"

    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/perception/local_occupancy_grid", latching_qos);

    bottom_clusters_sub_ = create_subscription<cluster_msgs::msg::ClusterPointsArray>(
        "/all_bottom_clusters", 10,
        std::bind(&BottomClustersToOccupancyGridNode::clustersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Node started with resolution=%.2f, size=%d", grid_resolution_, grid_size_);
}

void BottomClustersToOccupancyGridNode::updateLogOdds(int idx, bool occupied)
{
    log_odds_grid_[idx] += occupied ? l_occ : l_free;
    log_odds_grid_[idx] = std::clamp(log_odds_grid_[idx], -5.0f, 5.0f);
}

void BottomClustersToOccupancyGridNode::clustersCallback(
    const cluster_msgs::msg::ClusterPointsArray::SharedPtr msg)
{
    auto now = get_clock()->now();
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    occupancy_grid.header.stamp = now;
    occupancy_grid.header.frame_id = frame_id;
    occupancy_grid.info = current_grid_.info;
    occupancy_grid.info.origin.orientation.w = 1.0;

    start_x = grid_size_ / 2;
    start_y = grid_size_ / 2;

    std::vector<int8_t> grid_data(grid_size_ * grid_size_, 0);
    std::vector<float> temp_log_odds = log_odds_grid_;

    cv::Mat mask = cv::Mat::zeros(grid_size_, grid_size_, CV_8U);
    for (const auto &cluster : msg->clusters)
    {
        std::vector<cv::Point> pts;
        for (auto &pt : cluster.bottom_points_array)
        {
            int xi = static_cast<int>((pt.x - occupancy_grid.info.origin.position.x) / grid_resolution_);
            int yi = static_cast<int>((pt.y - occupancy_grid.info.origin.position.y) / grid_resolution_);
            if (xi>=0 && xi<grid_size_ && yi>=0 && yi<grid_size_)
                pts.emplace_back(xi, yi);
        }
        if (!pts.empty())
        {
            std::vector<cv::Point> hull;
            cv::convexHull(pts, hull);
            std::vector<std::vector<cv::Point>> hulls = {hull};
            cv::fillPoly(mask, hulls, 255);
        }
    }

    for (int y = 0; y < grid_size_; ++y)
    {
        for (int x = 0; x < grid_size_; ++x)
        {
            int idx = y * grid_size_ + x;
            bool occupied = mask.at<uint8_t>(y, x) > 0;
            updateLogOdds(idx, occupied);
            temp_log_odds[idx] = log_odds_grid_[idx];
            float odds = std::exp(temp_log_odds[idx]);
            float prob = odds / (1.0f + odds);
            grid_data[idx] = (prob > MaxProbability) ? 100
                            : (prob < MinProbability) ? 0
                            : 50;
        }
    }

    grid_data[start_y * grid_size_ + start_x] = 10; // Mark the center of the grid
    log_odds_grid_ = temp_log_odds;
    grid_data_global = grid_data;

    occupancy_grid.data = grid_data;
    occupancy_grid_pub_->publish(occupancy_grid);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BottomClustersToOccupancyGridNode>());
    rclcpp::shutdown();
    return 0;
}

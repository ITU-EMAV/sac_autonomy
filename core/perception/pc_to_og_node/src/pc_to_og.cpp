#include "pc_to_og_node/pc_to_og.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>
#include <limits>
#include <cstring>

using std::placeholders::_1;

PointCloudToOccupancyGrid::PointCloudToOccupancyGrid()
: Node("pc2og")
{
    // Declare parameters
    this->declare_parameter<double>("resolution", 0.05);
    this->declare_parameter<double>("grid_width", 30.0);
    this->declare_parameter<double>("grid_height", 30.0);
    this->declare_parameter<bool>("expand_occupied_cells", true);
    this->declare_parameter<std::string>("pointcloud_topic", "/zed/pointcloud_0");
    this->declare_parameter<std::string>("occupancy_topic", "/occupancy_grid");
    this->declare_parameter<double>("min_z", 0.1);

    // Get parameters
    resolution_ = this->get_parameter("resolution").get_value<double>();
    grid_width_m_ = this->get_parameter("grid_width").get_value<double>();
    grid_height_m_ = this->get_parameter("grid_height").get_value<double>();
    expand_occupied_cells_ = this->get_parameter("expand_occupied_cells").get_value<bool>();
    std::string pointcloud_topic = this->get_parameter("pointcloud_topic").get_value<std::string>();
    std::string occupancy_topic = this->get_parameter("occupancy_topic").get_value<std::string>();
    MIN_Z = this->get_parameter("min_z").get_value<double>();

    RCLCPP_INFO(this->get_logger(), " Params: resolution: %f, grid_width: %f, grid_height: %f, expand_occupied_cells: %d, pointcloud_topic: %s, occupancy_topic: %s, min_z: %f",
    resolution_, grid_width_m_, grid_height_m_, expand_occupied_cells_, pointcloud_topic.c_str(), occupancy_topic.c_str(), MIN_Z);

    // Create subscription and publisher using parameters.
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic, rclcpp::QoS(1),
    std::bind(&PointCloudToOccupancyGrid::pointcloud_callback, this, _1));

    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(occupancy_topic, 10);

    // Calculate grid cell dimensions
    grid_width_cells_ = static_cast<int>(grid_width_m_ / resolution_);
    grid_height_cells_ = static_cast<int>(grid_height_m_ / resolution_);

    // Initialize occupancy grid message
    occupancy_grid_msg_.header.frame_id = "base_link";
    occupancy_grid_msg_.info.resolution = resolution_;
    occupancy_grid_msg_.info.width = grid_width_cells_;
    occupancy_grid_msg_.info.height = grid_height_cells_;
    occupancy_grid_msg_.info.origin.position.x = -grid_width_m_ / 2.0;
    occupancy_grid_msg_.info.origin.position.y = -grid_height_m_ / 2.0;
    occupancy_grid_msg_.info.origin.position.z = 0.0;
    // Orientation remains identity (0,0,0,1)

    // Create dilation kernel if we are expanding occupied cells.
    if (expand_occupied_cells_) {
        int kernel_size = static_cast<int>(1.0 / resolution_);
        dilation_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    }

    RCLCPP_INFO(this->get_logger(), "PCL to OccupancyGrid node started.");
}

void PointCloudToOccupancyGrid::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Create an empty occupancy grid (initialize with free space: 0).
  cv::Mat grid(cv::Size(grid_width_cells_, grid_height_cells_), CV_8UC1, cv::Scalar(0));

  // Cache origin values for efficiency.
  const double origin_x = occupancy_grid_msg_.info.origin.position.x;
  const double origin_y = occupancy_grid_msg_.info.origin.position.y;

  // Use iterators to extract x, y, z fields.
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    // Filter out invalid points and ground points.
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= MIN_Z)
      continue;

    // Transform point to occupancy grid coordinates.
    float shifted_x = x - origin_x;
    float shifted_y = y - origin_y;
    int cell_x = static_cast<int>(std::floor(shifted_x / resolution_));
    int cell_y = static_cast<int>(std::floor(shifted_y / resolution_));

    // Check bounds and mark as occupied if valid.
    if (cell_x >= 0 && cell_x < grid_width_cells_ &&
        cell_y >= 0 && cell_y < grid_height_cells_)
    {
      grid.at<uint8_t>(cell_y, cell_x) = 100;
    }
  }

  // Optionally expand occupied cells using dilation.
  if (expand_occupied_cells_)
  {
    cv::Mat binary;
    // Convert grid to binary (occupied = 1, free = 0)
    cv::threshold(grid, binary, 50, 1, cv::THRESH_BINARY);
    cv::Mat dilated;
    try {
      cv::dilate(binary, dilated, dilation_kernel_);

    } catch (const cv::Exception& e){
      RCLCPP_ERROR(this->get_logger()," OpenCV exeption: %s", e.what());
    }

    // Multiply the binary image to get occupancy values (0 or 100) in one step.
    grid = dilated * 100;
  }

  // Copy the grid data to the occupancy grid message.
  size_t grid_size = grid.total() * sizeof(int8_t);
  occupancy_grid_msg_.data.resize(grid.total());
  std::memcpy(occupancy_grid_msg_.data.data(), grid.data, grid_size);

  // occupancy_grid_msg_.header.stamp = this->now();
  auto system_now = std::chrono::system_clock::now();
  auto system_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(system_now.time_since_epoch()).count();

  // Convert to ROS2 time format
  
  // occupancy_grid_msg_.header.stamp= rclcpp::Time(system_ns);

  // rclcpp::Time occupancy_time = occupancy_grid_msg_.header.stamp ;
  // rclcpp::Time pointcloud_time = msg->header.stamp;

  // Compute the duration (occupancy_time - pointcloud_time)
  // rclcpp::Duration diff = occupancy_time - pointcloud_time;
  // double diff_in_seconds = diff.seconds();

  // RCLCPP_INFO(this->get_logger(), "Time difference = %f seconds", diff_in_seconds);

  occupancy_grid_pub_->publish(occupancy_grid_msg_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToOccupancyGrid>());
  rclcpp::shutdown();
  return 0;
}

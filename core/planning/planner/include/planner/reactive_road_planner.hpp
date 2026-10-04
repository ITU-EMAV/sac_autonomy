#ifndef PLANNER__REACTIVE_ROAD_PLANNER_HPP_
#define PLANNER__REACTIVE_ROAD_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/path.hpp>

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace smart_car
{

class ReactiveRoadPlanner : public rclcpp::Node
{
public:
  explicit ReactiveRoadPlanner(const rclcpp::NodeOptions & options);

private:
  void maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void laneMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  bool buildHomography(int width, int height);
  double previousLateralAt(double forward) const;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr lane_mask_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::string output_frame_;
  int row_step_{12};
  int threshold_{127};
  int min_segment_width_px_{20};
  double min_forward_m_{0.5};
  double max_forward_m_{12.0};
  double smoothing_alpha_{0.35};
  double temporal_alpha_{0.25};
  double max_lateral_change_m_{0.5};
  double lane_center_weight_{0.65};
  std::vector<double> source_trapezoid_;
  std::vector<double> ground_trapezoid_;
  cv::Mat homography_;
  int homography_width_{0};
  int homography_height_{0};
  cv::Mat latest_lane_mask_;
  std::vector<cv::Point2f> previous_ground_path_;
};

}  // namespace smart_car

#endif  // PLANNER__REACTIVE_ROAD_PLANNER_HPP_

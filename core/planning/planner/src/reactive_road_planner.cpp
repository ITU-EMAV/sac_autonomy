#include "reactive_road_planner.hpp"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace smart_car
{

ReactiveRoadPlanner::ReactiveRoadPlanner(const rclcpp::NodeOptions & options)
: Node("reactive_road_planner_exe", options)
{
  declare_parameter<std::string>("drivable_mask_topic", "/perception/road/drivable_mask");
  declare_parameter<std::string>("lane_mask_topic", "/perception/road/lane_mask");
  declare_parameter<std::string>("trajectory_topic", "/reactive_planner/trajectory");
  declare_parameter<std::string>("output_frame", "base_link");
  declare_parameter<int>("row_step", 12);
  declare_parameter<int>("mask_threshold", 127);
  declare_parameter<int>("min_segment_width_px", 20);
  declare_parameter<double>("min_forward_m", 0.5);
  declare_parameter<double>("max_forward_m", 12.0);
  declare_parameter<double>("smoothing_alpha", 0.35);
  declare_parameter<double>("temporal_alpha", 0.25);
  declare_parameter<double>("max_lateral_change_m", 0.5);
  declare_parameter<double>("lane_center_weight", 0.65);
  declare_parameter<std::vector<double>>(
    "source_trapezoid", {0.35, 0.55, 0.65, 0.55, 0.95, 0.98, 0.05, 0.98});
  declare_parameter<std::vector<double>>(
    "ground_trapezoid", {12.0, 3.0, 12.0, -3.0, 0.5, -2.0, 0.5, 2.0});

  output_frame_ = get_parameter("output_frame").as_string();
  row_step_ = std::max(1, static_cast<int>(get_parameter("row_step").as_int()));
  threshold_ = static_cast<int>(get_parameter("mask_threshold").as_int());
  min_segment_width_px_ = std::max(
    1, static_cast<int>(get_parameter("min_segment_width_px").as_int()));
  min_forward_m_ = get_parameter("min_forward_m").as_double();
  max_forward_m_ = get_parameter("max_forward_m").as_double();
  smoothing_alpha_ = std::clamp(get_parameter("smoothing_alpha").as_double(), 0.0, 1.0);
  temporal_alpha_ = std::clamp(get_parameter("temporal_alpha").as_double(), 0.0, 1.0);
  max_lateral_change_m_ = std::max(0.0, get_parameter("max_lateral_change_m").as_double());
  lane_center_weight_ = std::clamp(get_parameter("lane_center_weight").as_double(), 0.0, 1.0);
  source_trapezoid_ = get_parameter("source_trapezoid").as_double_array();
  ground_trapezoid_ = get_parameter("ground_trapezoid").as_double_array();

  if (source_trapezoid_.size() != 8 || ground_trapezoid_.size() != 8) {
    throw std::invalid_argument("source_trapezoid and ground_trapezoid must contain 8 values");
  }

  auto qos = rclcpp::SensorDataQoS().keep_last(1);
  mask_sub_ = create_subscription<sensor_msgs::msg::Image>(
    get_parameter("drivable_mask_topic").as_string(), qos,
    std::bind(&ReactiveRoadPlanner::maskCallback, this, std::placeholders::_1));
  lane_mask_sub_ = create_subscription<sensor_msgs::msg::Image>(
    get_parameter("lane_mask_topic").as_string(), qos,
    std::bind(&ReactiveRoadPlanner::laneMaskCallback, this, std::placeholders::_1));
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    get_parameter("trajectory_topic").as_string(), rclcpp::QoS(1));

  RCLCPP_INFO(get_logger(), "Reactive road planner enabled (localization-free mode)");
}

void ReactiveRoadPlanner::laneMaskCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try {
    latest_lane_mask_ = cv_bridge::toCvShare(
      msg, sensor_msgs::image_encodings::MONO8)->image.clone();
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "Lane-mask conversion failed: %s", error.what());
  }
}

double ReactiveRoadPlanner::previousLateralAt(double forward) const
{
  if (previous_ground_path_.empty()) {
    return 0.0;
  }
  const auto nearest = std::min_element(
    previous_ground_path_.begin(), previous_ground_path_.end(),
    [forward](const auto & lhs, const auto & rhs) {
      return std::abs(lhs.x - forward) < std::abs(rhs.x - forward);
    });
  return nearest->y;
}

bool ReactiveRoadPlanner::buildHomography(int width, int height)
{
  if (width <= 1 || height <= 1) {
    return false;
  }
  std::vector<cv::Point2f> src;
  std::vector<cv::Point2f> dst;
  src.reserve(4);
  dst.reserve(4);
  for (std::size_t i = 0; i < 4; ++i) {
    src.emplace_back(
      static_cast<float>(source_trapezoid_[2 * i] * (width - 1)),
      static_cast<float>(source_trapezoid_[2 * i + 1] * (height - 1)));
    dst.emplace_back(
      static_cast<float>(ground_trapezoid_[2 * i]),
      static_cast<float>(ground_trapezoid_[2 * i + 1]));
  }
  homography_ = cv::getPerspectiveTransform(src, dst);
  homography_width_ = width;
  homography_height_ = height;
  return !homography_.empty();
}

void ReactiveRoadPlanner::maskCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv::Mat mask;
  try {
    mask = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Mask conversion failed: %s", error.what());
    return;
  }

  if ((mask.cols != homography_width_ || mask.rows != homography_height_) &&
    !buildHomography(mask.cols, mask.rows))
  {
    return;
  }

  std::vector<cv::Point2f> image_centers;
  const int top_row = std::clamp(
    static_cast<int>(source_trapezoid_[1] * (mask.rows - 1)), 0, mask.rows - 1);
  const int bottom_row = std::clamp(
    static_cast<int>(source_trapezoid_[7] * (mask.rows - 1)), 0, mask.rows - 1);
  double previous_center = 0.5 * static_cast<double>(mask.cols - 1);

  for (int row = bottom_row; row >= top_row; row -= row_step_) {
    const auto * pixels = mask.ptr<uint8_t>(row);
    int best_left = -1;
    int best_right = -1;
    double best_score = -1.0;
    for (int col = 0; col < mask.cols;) {
      while (col < mask.cols && pixels[col] <= threshold_) {
        ++col;
      }
      const int left = col;
      while (col < mask.cols && pixels[col] > threshold_) {
        ++col;
      }
      const int right = col - 1;
      const int width = right - left + 1;
      if (width < min_segment_width_px_) {
        continue;
      }
      const double center = 0.5 * static_cast<double>(left + right);
      const double score = static_cast<double>(width) - 0.5 * std::abs(center - previous_center);
      if (score > best_score) {
        best_score = score;
        best_left = left;
        best_right = right;
      }
    }
    if (best_left >= 0) {
      double road_center = 0.5 * static_cast<double>(best_left + best_right);

      // When two lane markings are visible inside the selected drivable region,
      // use their midpoint. This keeps the path in its lane when an intersection
      // makes the drivable-area mask suddenly much wider.
      if (!latest_lane_mask_.empty() && latest_lane_mask_.size() == mask.size()) {
        const auto * lane_pixels = latest_lane_mask_.ptr<uint8_t>(row);
        std::vector<double> markings;
        for (int col = best_left; col <= best_right;) {
          while (col <= best_right && lane_pixels[col] <= threshold_) {
            ++col;
          }
          const int left = col;
          while (col <= best_right && lane_pixels[col] > threshold_) {
            ++col;
          }
          if (col > left) {
            markings.push_back(0.5 * static_cast<double>(left + col - 1));
          }
        }
        double left_mark = -1.0;
        double right_mark = -1.0;
        for (const double marking : markings) {
          if (marking < previous_center && (left_mark < 0.0 || marking > left_mark)) {
            left_mark = marking;
          } else if (marking >= previous_center &&
            (right_mark < 0.0 || marking < right_mark))
          {
            right_mark = marking;
          }
        }
        if (left_mark >= 0.0 && right_mark >= 0.0 && right_mark - left_mark > 8.0) {
          const double lane_center = 0.5 * (left_mark + right_mark);
          road_center = lane_center_weight_ * lane_center +
            (1.0 - lane_center_weight_) * road_center;
        }
      }
      previous_center = road_center;
      image_centers.emplace_back(static_cast<float>(previous_center), static_cast<float>(row));
    }
  }

  if (image_centers.size() < 2) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No usable drivable corridor in mask");
    return;
  }

  std::vector<cv::Point2f> ground;
  cv::perspectiveTransform(image_centers, ground, homography_);
  ground.erase(
    std::remove_if(ground.begin(), ground.end(), [this](const cv::Point2f & point) {
      return !std::isfinite(point.x) || !std::isfinite(point.y) ||
             point.x < min_forward_m_ || point.x > max_forward_m_;
    }), ground.end());
  std::sort(ground.begin(), ground.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.x < rhs.x;
  });
  if (ground.size() < 2) {
    return;
  }

  for (std::size_t i = 1; i < ground.size(); ++i) {
    ground[i].y = static_cast<float>(
      smoothing_alpha_ * ground[i].y + (1.0 - smoothing_alpha_) * ground[i - 1].y);
  }

  // Preserve heading continuity between frames. In a wide intersection this
  // biases the path toward its previous direction (straight by default) instead
  // of switching randomly between equally valid branches.
  if (!previous_ground_path_.empty()) {
    for (auto & point : ground) {
      const double previous_y = previousLateralAt(point.x);
      const double proposed_y = temporal_alpha_ * point.y + (1.0 - temporal_alpha_) * previous_y;
      point.y = static_cast<float>(std::clamp(
        proposed_y, previous_y - max_lateral_change_m_, previous_y + max_lateral_change_m_));
    }
  }
  previous_ground_path_ = ground;

  nav_msgs::msg::Path path;
  path.header.stamp = msg->header.stamp;
  path.header.frame_id = output_frame_;
  path.poses.reserve(ground.size() + 1);

  geometry_msgs::msg::PoseStamped origin;
  origin.header = path.header;
  origin.pose.orientation.w = 1.0;
  path.poses.push_back(origin);
  for (std::size_t i = 0; i < ground.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = ground[i].x;
    pose.pose.position.y = ground[i].y;
    const auto & next = ground[std::min(i + 1, ground.size() - 1)];
    const auto & prev = ground[i == 0 ? 0 : i - 1];
    const double yaw = std::atan2(next.y - prev.y, next.x - prev.x);
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    path.poses.push_back(std::move(pose));
  }
  path_pub_->publish(path);
}

}  // namespace smart_car

RCLCPP_COMPONENTS_REGISTER_NODE(smart_car::ReactiveRoadPlanner)

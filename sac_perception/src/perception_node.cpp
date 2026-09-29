#include "sac_perception/perception_node.hpp"

#include <chrono>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>

namespace sac_perception
{

namespace
{
rclcpp::NodeOptions withParameters(const rclcpp::NodeOptions & options)
{
  rclcpp::NodeOptions o(options);
  o.allow_undeclared_parameters(true);
  o.automatically_declare_parameters_from_overrides(true);
  return o;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, double value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  char text[32];
  std::snprintf(text, sizeof(text), "%.2f", value);
  kv.value = text;
  return kv;
}
}  // namespace

PerceptionNode::PerceptionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("perception", withParameters(options)),
  root_(this, ""),
  filter_loader_("sac_perception", "sac_perception::PointFilter"),
  source_loader_("sac_perception", "sac_perception::GridSource")
{
  grid_frame_ = root_.getString("grid_frame", "odom");
  base_frame_ = root_.getString("base_frame", "base_footprint");
  occupied_threshold_ = static_cast<float>(root_.getDouble("occupied_threshold", occupied_threshold_));
  grid_.grid = std::make_unique<RollingGrid>(
    root_.getDouble("grid.size", 80.0), root_.getDouble("grid.resolution", 0.2));
  grid_.grid->setRecenterDistance(root_.getDouble("grid.recenter_distance", 2.0));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  SourceContext context;
  context.node = this;
  context.tf = tf_buffer_;
  context.grid_frame = grid_frame_;
  context.base_frame = base_frame_;
  context.grid = &grid_;
  context.filters = &filter_loader_;
  for (const std::string & name : root_.getStrings("sources.names", {})) {
    auto params = std::make_shared<RosParams>(this, "sources." + name + ".");
    const std::string type = params->getString("type", "");
    if (type.empty()) {
      throw std::invalid_argument("sources." + name + ".type is missing");
    }
    auto source = source_loader_.createSharedInstance(type);
    source->initialize(context, name, params);
    sources_.push_back(source);
  }

  grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/grid", rclcpp::QoS(1));
  timing_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/timing", rclcpp::QoS(1));
  const double rate = root_.getDouble("rate", 20.0);
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });
  RCLCPP_INFO(
    get_logger(), "Grid %.0f m at %.2f m in %s, %zu sources", root_.getDouble("grid.size", 80.0),
    grid_.grid->resolution(), grid_frame_.c_str(), sources_.size());
}

void PerceptionNode::tick()
{
  // Messages that came before their TF
  for (const auto & source : sources_) {
    source->retry();
  }
  geometry_msgs::msg::TransformStamped base;
  try {
    base = tf_buffer_->lookupTransform(grid_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No %s -> %s: %s", grid_frame_.c_str(), base_frame_.c_str(), e.what());
    return;
  }
  const rclcpp::Time now = this->now();
  const auto start = std::chrono::steady_clock::now();
  nav_msgs::msg::OccupancyGrid message;
  {
    std::lock_guard<std::mutex> lock(grid_.mutex);
    RollingGrid & grid = *grid_.grid;
    grid.recenter(base.transform.translation.x, base.transform.translation.y);
    if (ticked_) {
      grid.decay(std::max(0.0, (now - last_tick_).seconds()));
    }
    message.info.resolution = static_cast<float>(grid.resolution());
    message.info.width = grid.width();
    message.info.height = grid.width();
    message.info.origin.position.x = grid.originX();
    message.info.origin.position.y = grid.originY();
    message.info.origin.position.z = base.transform.translation.z;  // drawn at the car's height
    message.info.origin.orientation.w = 1.0;
    message.data = grid.combined();
  }
  last_tick_ = now;
  ticked_ = true;
  message.header.stamp = base.header.stamp;
  message.header.frame_id = grid_frame_;
  message.info.map_load_time = message.header.stamp;
  grid_publisher_->publish(message);
  const double grid_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  grid_ms_max_ = std::max(grid_ms_max_, grid_ms);

  diagnostic_msgs::msg::DiagnosticArray timing;
  timing.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus grid_status;
  grid_status.name = "grid";
  grid_status.values = {keyValue("last_ms", grid_ms), keyValue("max_ms", grid_ms_max_)};
  timing.status.push_back(grid_status);
  for (const auto & source : sources_) {
    const SourceTiming t = source->timing();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = source->name();
    status.values = {
      keyValue("last_ms", t.last_ms), keyValue("mean_ms", t.count ? t.sum_ms / t.count : 0.0),
      keyValue("max_ms", t.max_ms), keyValue("messages", static_cast<double>(t.count)),
      keyValue("skipped", static_cast<double>(t.skipped))};
    for (const auto & [key, value] : source->diagnostics()) {
      status.values.push_back(keyValue(key, value));
    }
    timing.status.push_back(status);
  }
  timing_publisher_->publish(timing);
}

}  // namespace sac_perception

RCLCPP_COMPONENTS_REGISTER_NODE(sac_perception::PerceptionNode)

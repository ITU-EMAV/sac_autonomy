#include "sac_perception/perception_node.hpp"

#include <chrono>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>

#include "sac_perception/robot_geometry.hpp"

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
  map_loader_("sac_perception", "sac_perception::MapRepresentation"),
  filter_loader_("sac_perception", "sac_perception::PointFilter"),
  source_loader_("sac_perception", "sac_perception::GridSource")
{
  grid_frame_ = root_.getString("grid_frame", "odom");
  base_frame_ = root_.getString("base_frame", "base_footprint");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/grid", rclcpp::QoS(1));
  timing_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/timing", rclcpp::QoS(1));

  const std::string from = root_.getString("vehicle.from", "none");
  if (from == "robot_description") {
    // Latched by robot_state_publisher; the sources start when it comes
    const std::string topic = root_.getString("vehicle.topic", "/robot_description");
    description_subscription_ = create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).transient_local().reliable(),
      [this, topic](const std_msgs::msg::String::ConstSharedPtr & message) {
        if (map_.map) {
          return;  // started already
        }
        const auto geometry = vehicleFromUrdf(message->data, base_frame_);
        if (!geometry) {
          RCLCPP_ERROR(get_logger(), "No %s in the URDF on %s, or no shapes", base_frame_.c_str(), topic.c_str());
          return;
        }
        for (const std::string & warning : geometry->warnings) {
          RCLCPP_WARN(get_logger(), "Car's box: mesh not read, its origin counted: %s", warning.c_str());
        }
        start(geometry->box);
      });
    RCLCPP_INFO(get_logger(), "Waiting for the car's URDF on %s", topic.c_str());
  } else if (from == "box") {
    const std::vector<double> b = root_.getDoubles("vehicle.box", {});
    if (b.size() != 6) {
      throw std::invalid_argument("vehicle.box: [min x, y, z, max x, y, z]");
    }
    VehicleBox vehicle;
    vehicle.min = Eigen::Vector3f(b[0], b[1], b[2]);
    vehicle.max = Eigen::Vector3f(b[3], b[4], b[5]);
    vehicle.known = true;
    start(vehicle);
  } else if (from == "none") {
    start(VehicleBox{});
  } else {
    throw std::invalid_argument("vehicle.from: robot_description, box or none, not '" + from + "'");
  }
  const double rate = root_.getDouble("rate", 20.0);
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });
}

void PerceptionNode::start(const VehicleBox & vehicle)
{
  vehicle_ = vehicle;
  if (vehicle.known) {
    RCLCPP_INFO(
      get_logger(), "Car's box in %s: x %.2f..%.2f, y %.2f..%.2f, z %.2f..%.2f m", base_frame_.c_str(),
      vehicle.min.x(), vehicle.max.x(), vehicle.min.y(), vehicle.max.y(), vehicle.min.z(), vehicle.max.z());
  }
  GridGeometry geometry;
  geometry.size = root_.getDouble("grid.size", geometry.size);
  geometry.resolution = root_.getDouble("grid.resolution", geometry.resolution);
  geometry.recenter_distance = root_.getDouble("grid.recenter_distance", geometry.recenter_distance);
  const std::string type = root_.getString("map.type", "direct_projection");
  auto map = map_loader_.createSharedInstance(type);
  map->initialize(RosParams(this, "map."), geometry, vehicle);

  SourceContext context;
  context.node = this;
  context.tf = tf_buffer_;
  context.grid_frame = grid_frame_;
  context.base_frame = base_frame_;
  context.map = &map_;
  context.filters = &filter_loader_;
  context.vehicle = vehicle;
  {
    std::lock_guard<std::mutex> lock(map_.mutex);
    map_.map = map;
  }
  for (const std::string & name : root_.getStrings("sources.names", {})) {
    auto params = std::make_shared<RosParams>(this, "sources." + name + ".");
    const std::string source_type = params->getString("type", "");
    if (source_type.empty()) {
      throw std::invalid_argument("sources." + name + ".type is missing");
    }
    auto source = source_loader_.createSharedInstance(source_type);
    source->initialize(context, name, params);
    sources_.push_back(source);
  }
  RCLCPP_INFO(
    get_logger(), "Grid %.0f m at %.2f m in %s, map %s, %zu sources", geometry.size, geometry.resolution,
    grid_frame_.c_str(), type.c_str(), sources_.size());
}

void PerceptionNode::tick()
{
  if (!map_.map) {
    return;  // no grid before the sources run: the planner stops on its own
  }
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
  std::vector<std::pair<std::string, double>> map_diagnostics;
  {
    std::lock_guard<std::mutex> lock(map_.mutex);
    MapRepresentation & map = *map_.map;
    map.recenter(base.transform.translation.x, base.transform.translation.y);
    if (ticked_) {
      map.decay(std::max(0.0, (now - last_tick_).seconds()));
    }
    message.info.resolution = static_cast<float>(map.resolution());
    message.info.width = map.width();
    message.info.height = map.width();
    message.info.origin.position.x = map.originX();
    message.info.origin.position.y = map.originY();
    message.info.origin.position.z = base.transform.translation.z;  // drawn at the car's height
    message.info.origin.orientation.w = 1.0;
    message.data = map.project();
    map_diagnostics = map.diagnostics();
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
  for (const auto & [key, value] : map_diagnostics) {
    grid_status.values.push_back(keyValue(key, value));
  }
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

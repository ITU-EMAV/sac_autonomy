#include "sac_perception/perception_node.hpp"

#include <chrono>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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
  clusterer_loader_("sac_perception", "sac_perception::Clusterer"),
  tracker_loader_("sac_perception", "sac_perception::Tracker"),
  source_loader_("sac_perception", "sac_perception::GridSource")
{
  grid_frame_ = root_.getString("grid_frame", "odom");
  base_frame_ = root_.getString("base_frame", "base_footprint");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/grid", rclcpp::QoS(1));
  timing_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/timing", rclcpp::QoS(1));
  if (root_.getBool("objects.enabled", true)) {
    objects_window_ = root_.getDouble("objects.window", objects_window_);
    objects_range_ = root_.getDouble("objects.max_range", objects_range_);
    clusterer_ = clusterer_loader_.createSharedInstance(
      root_.getString("objects.clusterer.type", "connected_components"));
    clusterer_->initialize(RosParams(this, "objects.clusterer."));
    tracker_ = tracker_loader_.createSharedInstance(root_.getString("objects.tracker.type", "kalman_tracker"));
    tracker_->initialize(RosParams(this, "objects.tracker."));
    objects_publisher_ = create_publisher<sac_perception_msgs::msg::TrackedObjects>("~/objects", rclcpp::QoS(1));
    markers_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/objects/markers", rclcpp::QoS(1));
  }
  if (root_.getBool("publish_map", false)) {
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/map", rclcpp::QoS(1));
    map_period_ = 1.0 / root_.getDouble("publish_map_rate", 5.0);
  }

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
  double latest = 0.0;
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
    if (tracker_) {
      latest = map.latest();
      map.recent(latest - objects_window_, recent_);
    }
  }
  last_tick_ = now;
  ticked_ = true;
  message.header.stamp = base.header.stamp;
  message.header.frame_id = grid_frame_;
  message.info.map_load_time = message.header.stamp;
  grid_publisher_->publish(message);
  const double grid_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  grid_ms_max_ = std::max(grid_ms_max_, grid_ms);
  double map_ms = -1.0;
  if (map_publisher_ && (last_map_.nanoseconds() == 0 || (now - last_map_).seconds() >= map_period_ - 1e-3)) {
    last_map_ = now;
    const auto map_start = std::chrono::steady_clock::now();
    publishMap(message.header.stamp);
    map_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - map_start).count();
    map_ms_last_ = map_ms;
  }

  if (tracker_ && latest > 0.0) {
    const auto objects_start = std::chrono::steady_clock::now();
    updateObjects(
      recent_, latest, message.header.stamp,
      Eigen::Vector3d(base.transform.translation.x, base.transform.translation.y, base.transform.translation.z));
    objects_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - objects_start).count();
  }

  diagnostic_msgs::msg::DiagnosticArray timing;
  timing.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus grid_status;
  grid_status.name = "grid";
  grid_status.values = {keyValue("last_ms", grid_ms), keyValue("max_ms", grid_ms_max_)};
  if (map_publisher_) {
    grid_status.values.push_back(keyValue("map_publish_ms", map_ms_last_));
  }
  if (tracker_) {
    grid_status.values.push_back(keyValue("objects_ms", objects_ms_));
    grid_status.values.push_back(keyValue("objects", static_cast<double>(tracker_->tracks().size())));
  }
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

void PerceptionNode::updateObjects(
  std::vector<RecentPoint> & recent, double time, const rclcpp::Time & stamp, const Eigen::Vector3d & car)
{
  const double z = car.z();
  const Eigen::Vector2f centre = car.head<2>().cast<float>();
  const float range2 = static_cast<float>(objects_range_ * objects_range_);
  recent.erase(
    std::remove_if(
      recent.begin(), recent.end(), [&](const RecentPoint & p) { return (p.xy - centre).squaredNorm() > range2; }),
    recent.end());
  clusterer_->cluster(recent, clusters_);
  tracker_->update(clusters_, time);
  sac_perception_msgs::msg::TrackedObjects objects;
  objects.header.stamp = stamp;
  objects.header.frame_id = grid_frame_;
  visualization_msgs::msg::MarkerArray markers;
  std::size_t count = 0;
  for (const Track & t : tracker_->tracks()) {
    sac_perception_msgs::msg::TrackedObject o;
    o.id = t.id;
    o.classification = static_cast<uint8_t>(t.last.classification);
    o.confirmed = t.confirmed;
    o.moving = t.moving;
    o.position.x = t.x(0);
    o.position.y = t.x(1);
    o.position.z = z;
    o.velocity.x = t.x(2);
    o.velocity.y = t.x(3);
    o.position_covariance = {t.P(0, 0), t.P(0, 1), t.P(1, 0), t.P(1, 1)};
    o.velocity_covariance = {t.P(2, 2), t.P(2, 3), t.P(3, 2), t.P(3, 3)};
    o.yaw = t.last.yaw;
    o.length = t.last.length;
    o.width = t.last.width;
    o.height = std::isfinite(t.last.height) ? t.last.height : 0.0f;
    o.age = static_cast<float>(time - t.first);
    o.hits = t.hits;
    objects.objects.push_back(o);
    if (!t.confirmed) {
      continue;
    }
    // Its box, coloured by class (moving: red), and its velocity
    visualization_msgs::msg::Marker box;
    box.header = objects.header;
    box.ns = "objects";
    box.id = static_cast<int>(count++);
    box.type = visualization_msgs::msg::Marker::CUBE;
    const float height = std::max(0.3f, o.height);
    box.pose.position.x = o.position.x;
    box.pose.position.y = o.position.y;
    box.pose.position.z = z + height / 2.0;
    box.pose.orientation.z = std::sin(o.yaw / 2.0);
    box.pose.orientation.w = std::cos(o.yaw / 2.0);
    box.scale.x = std::max(0.2f, o.length);
    box.scale.y = std::max(0.2f, o.width);
    box.scale.z = height;
    static const float colours[5][3] = {
      {0.6f, 0.6f, 0.6f}, {1.0f, 0.8f, 0.0f}, {0.2f, 0.5f, 1.0f}, {0.9f, 0.5f, 0.1f}, {0.4f, 0.4f, 0.4f}};
    const auto & c = colours[std::min<int>(o.classification, 4)];
    box.color.r = o.moving ? 1.0f : c[0];
    box.color.g = o.moving ? 0.1f : c[1];
    box.color.b = o.moving ? 0.1f : c[2];
    box.color.a = 0.5f;
    markers.markers.push_back(box);
    if (o.moving) {
      visualization_msgs::msg::Marker arrow = box;
      arrow.id = static_cast<int>(count++);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.points.resize(2);
      arrow.points[0].x = o.position.x;
      arrow.points[0].y = o.position.y;
      arrow.points[0].z = z + height;
      arrow.points[1].x = o.position.x + o.velocity.x;  // where it is in a second
      arrow.points[1].y = o.position.y + o.velocity.y;
      arrow.points[1].z = z + height;
      arrow.pose = geometry_msgs::msg::Pose();
      arrow.scale.x = 0.1;
      arrow.scale.y = 0.2;
      arrow.scale.z = 0.2;
      arrow.color.a = 1.0f;
      markers.markers.push_back(arrow);
    }
  }
  for (std::size_t k = count; k < markers_last_; ++k) {  // the ones of before that are gone
    visualization_msgs::msg::Marker gone;
    gone.header = objects.header;
    gone.ns = "objects";
    gone.id = static_cast<int>(k);
    gone.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(gone);
  }
  markers_last_ = count;
  objects_publisher_->publish(objects);
  markers_publisher_->publish(markers);
}

void PerceptionNode::publishMap(const rclcpp::Time & stamp)
{
  {
    std::lock_guard<std::mutex> lock(map_.mutex);
    map_.map->points(map_points_);
  }
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = stamp;
  cloud.header.frame_id = grid_frame_;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
    sensor_msgs::msg::PointField::FLOAT32, "occupancy", 1, sensor_msgs::msg::PointField::FLOAT32, "blocks", 1,
    sensor_msgs::msg::PointField::FLOAT32, "source", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(map_points_.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"), occupancy(cloud, "occupancy"),
  blocks(cloud, "blocks"), source(cloud, "source");
  for (const MapPoint & p : map_points_) {
    *x = p.position.x();
    *y = p.position.y();
    *z = p.position.z();
    *occupancy = p.occupancy;
    *blocks = p.blocks;
    *source = p.source;
    ++x, ++y, ++z, ++occupancy, ++blocks, ++source;
  }
  map_publisher_->publish(cloud);
}

}  // namespace sac_perception

RCLCPP_COMPONENTS_REGISTER_NODE(sac_perception::PerceptionNode)

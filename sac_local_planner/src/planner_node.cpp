#include "sac_local_planner/planner_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>

#include "sac_perception/grid.hpp"

namespace sac_local_planner
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

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, double value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  char text[32];
  std::snprintf(text, sizeof(text), "%.3f", value);
  kv.value = text;
  return kv;
}
}  // namespace

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("local_planner", withParameters(options)),
  root_(this, ""),
  generator_loader_("sac_local_planner", "sac_local_planner::TrajectoryGenerator"),
  cost_loader_("sac_local_planner", "sac_local_planner::CostFunction")
{
  map_frame_ = root_.getString("map_frame", "map");
  base_frame_ = root_.getString("base_frame", "base_footprint");
  occupied_threshold_ = static_cast<int>(root_.getDouble("occupied_threshold", occupied_threshold_));
  max_grid_age_ = root_.getDouble("max_grid_age", max_grid_age_);

  const sac_perception::RosParams generator_params(this, "generator.");
  auto generator = generator_loader_.createSharedInstance(generator_params.getString("type", "frenet_lattice"));
  generator->initialize(generator_params);
  std::vector<WeightedCost> costs;
  for (const std::string & name : root_.getStrings("costs.names", {})) {
    const sac_perception::RosParams p(this, "costs." + name + ".");
    WeightedCost w;
    w.name = name;
    w.weight = p.getDouble("weight", 1.0);
    w.function = cost_loader_.createSharedInstance(p.getString("type", name));
    w.function->initialize(p);
    costs.push_back(w);
  }
  Footprint footprint;
  footprint.offsets = root_.getDoubles("footprint.offsets", footprint.offsets);
  footprint.radius = root_.getDouble("footprint.radius", footprint.radius);
  footprint.safety_margin = root_.getDouble("footprint.safety_margin", footprint.safety_margin);
  SpeedLimits limits;
  limits.max_speed = root_.getDouble("limits.max_speed", limits.max_speed);
  limits.max_lateral_acceleration = root_.getDouble("limits.max_lateral_acceleration", limits.max_lateral_acceleration);
  limits.max_acceleration = root_.getDouble("limits.max_acceleration", limits.max_acceleration);
  limits.max_deceleration = root_.getDouble("limits.max_deceleration", limits.max_deceleration);
  limits.stop_margin = root_.getDouble("limits.stop_margin", limits.stop_margin);
  planner_ = std::make_unique<LocalPlanner>(generator, costs, footprint, limits);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  rclcpp::QoS latched(1);
  latched.transient_local();
  path_subscription_ = create_subscription<nav_msgs::msg::Path>(
    "path", latched, [this](const nav_msgs::msg::Path::ConstSharedPtr & m) { onPath(m); });
  grid_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "grid", rclcpp::QoS(1), [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & m) { onGrid(m); });
  trajectory_publisher_ = create_publisher<sac_planning_msgs::msg::Trajectory>("~/trajectory", rclcpp::QoS(1));
  candidates_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/candidates", rclcpp::QoS(1));
  timing_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/timing", rclcpp::QoS(1));
  const double rate = root_.getDouble("rate", 10.0);
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });
  RCLCPP_INFO(get_logger(), "Local planner: %zu cost functions, %.0f Hz", costs.size(), rate);
}

void LocalPlannerNode::onPath(const nav_msgs::msg::Path::ConstSharedPtr & message)
{
  if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
    RCLCPP_ERROR(get_logger(), "Path in '%s', expected '%s'", message->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  std::vector<Eigen::Vector2d> points;
  for (const auto & p : message->poses) {
    points.emplace_back(p.pose.position.x, p.pose.position.y);
  }
  route_.build(points);
  route_hint_ = -1.0;
  RCLCPP_INFO(get_logger(), "Route: %.0f m, %s", route_.length(), route_.closed() ? "loop" : "open");
}

void LocalPlannerNode::onGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & message)
{
  const auto start = std::chrono::steady_clock::now();
  const auto & info = message->info;
  if (info.width != info.height || info.width == 0) {
    return;  // the perception grid is square
  }
  std::vector<uint8_t> occupied(message->data.size());
  for (std::size_t k = 0; k < occupied.size(); ++k) {
    occupied[k] = message->data[k] >= occupied_threshold_ ? 1 : 0;
  }
  obstacles_.distance = sac_perception::distanceTransform(occupied, static_cast<int>(info.width), info.resolution);
  obstacles_.width = static_cast<int>(info.width);
  obstacles_.resolution = info.resolution;
  obstacles_.origin_x = info.origin.position.x;
  obstacles_.origin_y = info.origin.position.y;
  grid_frame_ = message->header.frame_id;
  grid_stamp_ = rclcpp::Time(message->header.stamp);
  edt_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void LocalPlannerNode::tick()
{
  if (route_.empty()) {
    return;
  }
  geometry_msgs::msg::TransformStamped base;
  try {
    base = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No pose: %s", e.what());
    return;
  }
  const rclcpp::Time stamp(base.header.stamp);

  // The car's speed from its pose over time (smoothed)
  const Eigen::Vector2d position(base.transform.translation.x, base.transform.translation.y);
  if (last_pose_stamp_) {
    const double dt = (stamp - *last_pose_stamp_).seconds();
    if (dt > 0.5 || dt < 0.0) {
      speed_ = 0.0;
    } else if (dt > 1e-3) {
      speed_ = 0.5 * speed_ + 0.5 * (position - last_position_).norm() / dt;
    }
  }
  if (!last_pose_stamp_ || (stamp - *last_pose_stamp_).seconds() > 1e-3) {
    last_pose_stamp_ = stamp;
    last_position_ = position;
  }

  PlanningContext context;
  context.route = &route_;
  context.ego.x = position.x();
  context.ego.y = position.y();
  context.ego.yaw = yawOf(base.transform.rotation);
  context.speed = speed_;
  const Eigen::Vector2d sd = route_.toFrenet(position, route_hint_);
  route_hint_ = sd.x();
  context.s = sd.x();
  context.d = sd.y();
  context.heading_error = wrapAngle(context.ego.yaw - route_.at(context.s).yaw);
  context.previous_target_d = previous_target_d_;
  context.has_previous = has_previous_;

  // No fresh grid: no blind driving
  if (!grid_stamp_ || (stamp - *grid_stamp_).seconds() > max_grid_age_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No recent obstacle grid: stopping");
    publishStop(context, stamp);
    return;
  }
  try {
    const auto t = tf_buffer_->lookupTransform(grid_frame_, map_frame_, tf2::TimePointZero);
    obstacles_.yaw = yawOf(t.transform.rotation);
    obstacles_.tx = t.transform.translation.x;
    obstacles_.ty = t.transform.translation.y;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No %s -> %s: %s", grid_frame_.c_str(), map_frame_.c_str(), e.what());
    publishStop(context, stamp);
    return;
  }
  context.obstacles = &obstacles_;

  const auto start = std::chrono::steady_clock::now();
  const PlanResult result = planner_->plan(context);
  const double plan_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  plan_ms_max_ = std::max(plan_ms_max_, plan_ms);
  if (result.chosen < 0) {
    publishStop(context, stamp);
    return;
  }
  previous_target_d_ = result.candidates[result.chosen].target_d;
  has_previous_ = true;
  publish(result, context, stamp);

  diagnostic_msgs::msg::DiagnosticArray timing;
  timing.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "local_planner";
  std::size_t free = 0;
  for (const Candidate & c : result.candidates) {
    free += c.free();
  }
  status.values = {
    keyValue("plan_ms", plan_ms), keyValue("plan_ms_max", plan_ms_max_), keyValue("distance_map_ms", edt_ms_),
    keyValue("candidates", static_cast<double>(result.candidates.size())),
    keyValue("free", static_cast<double>(free)),
    keyValue("target_d", previous_target_d_), keyValue("stopping", result.stopping ? 1.0 : 0.0),
    keyValue("speed", speed_), keyValue("d", context.d)};
  for (const auto & [name, value] : result.candidates[result.chosen].costs) {
    status.values.push_back(keyValue("cost." + name, value));
  }
  timing.status.push_back(status);
  timing_publisher_->publish(timing);
}

void LocalPlannerNode::publish(const PlanResult & result, const PlanningContext & context, const rclcpp::Time & stamp)
{
  (void)context;
  const Candidate & chosen = result.candidates[result.chosen];
  sac_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = map_frame_;
  trajectory.stopping = result.stopping;
  for (std::size_t i = 0; i < chosen.points.size(); ++i) {
    sac_planning_msgs::msg::TrajectoryPoint p;
    p.x = chosen.points[i].x;
    p.y = chosen.points[i].y;
    p.yaw = chosen.points[i].yaw;
    p.curvature = chosen.points[i].curvature;
    p.speed = result.speeds[i];
    p.s = static_cast<double>(i) * chosen.step;
    trajectory.points.push_back(p);
  }
  trajectory_publisher_->publish(trajectory);

  if (candidates_publisher_->get_subscription_count() == 0) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  for (std::size_t k = 0; k < result.candidates.size(); ++k) {
    const Candidate & c = result.candidates[k];
    const bool is_chosen = static_cast<int>(k) == result.chosen;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = is_chosen ? "chosen" : "candidates";
    m.id = static_cast<int>(k);
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.scale.x = is_chosen ? 0.25 : 0.04;
    m.pose.orientation.w = 1.0;
    if (is_chosen) {
      m.color.r = 0.1f, m.color.g = 0.4f, m.color.b = 1.0f, m.color.a = 1.0f;
    } else if (c.free()) {
      m.color.r = 0.2f, m.color.g = 0.9f, m.color.b = 0.2f, m.color.a = 0.35f;
    } else {
      m.color.r = 1.0f, m.color.g = 0.2f, m.color.b = 0.2f, m.color.a = 0.35f;
    }
    for (const Pose2 & p : c.points) {
      geometry_msgs::msg::Point q;
      q.x = p.x;
      q.y = p.y;
      q.z = 0.3;
      m.points.push_back(q);
    }
    markers.markers.push_back(m);
  }
  candidates_publisher_->publish(markers);
}

void LocalPlannerNode::publishStop(const PlanningContext & context, const rclcpp::Time & stamp)
{
  // Where the car is, speed 0
  sac_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = map_frame_;
  trajectory.stopping = true;
  for (int i = 0; i < 10; ++i) {
    const Pose2 p = route_.at(context.s + 0.5 * i);
    sac_planning_msgs::msg::TrajectoryPoint q;
    const Eigen::Vector2d xy = route_.toCartesian(context.s + 0.5 * i, context.d);
    q.x = xy.x();
    q.y = xy.y();
    q.yaw = p.yaw;
    q.speed = 0.0;
    q.s = 0.5 * i;
    trajectory.points.push_back(q);
  }
  trajectory_publisher_->publish(trajectory);
}

}  // namespace sac_local_planner

RCLCPP_COMPONENTS_REGISTER_NODE(sac_local_planner::LocalPlannerNode)

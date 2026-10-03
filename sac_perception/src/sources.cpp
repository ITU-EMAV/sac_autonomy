#include "sac_perception/sources.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace sac_perception
{

namespace
{
using Clock = std::chrono::steady_clock;

double millisecondsSince(Clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

Eigen::Isometry3f toIsometry(const geometry_msgs::msg::TransformStamped & t)
{
  return tf2::transformToEigen(t).cast<float>();
}
}  // namespace

// ---------------------------------------------------------------- common
void GridSource::initialize(const SourceContext & context, const std::string & name, std::shared_ptr<RosParams> params)
{
  context_ = context;
  name_ = name;
  params_ = std::move(params);
  max_age_ = params_->getDouble("max_age", max_age_);
  provides_ground_ = params_->getBool("provides_ground", provides_ground_);
  ground_margin_ = params_->getDouble("ground_margin", ground_margin_);
  ground_search_radius_ = params_->getDouble("ground_search_radius", ground_search_radius_);
  ground_max_age_ = params_->getDouble("ground_max_age", ground_max_age_);
  max_mark_range_ = params_->getDouble("max_mark_range", max_mark_range_);
  if (params_->has("mount")) {
    const std::vector<double> m = params_->getDoubles("mount", {});
    if (m.size() != 6) {
      throw std::invalid_argument(params_->prefix() + "mount: [x, y, z, roll, pitch, yaw]");
    }
    Eigen::Isometry3f mount = Eigen::Isometry3f::Identity();
    mount.translation() = Eigen::Vector3f(m[0], m[1], m[2]);
    mount.linear() = (Eigen::AngleAxisf(m[5], Eigen::Vector3f::UnitZ()) *
      Eigen::AngleAxisf(m[4], Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(m[3], Eigen::Vector3f::UnitX())).toRotationMatrix();
    mount_ = mount;
  }
  LayerParams layer;
  layer.hit = static_cast<float>(params_->getDouble("hit", layer.hit));
  layer.miss = static_cast<float>(params_->getDouble("miss", layer.miss));
  layer.min = static_cast<float>(params_->getDouble("min", layer.min));
  layer.max = static_cast<float>(params_->getDouble("max", layer.max));
  layer.decay = static_cast<float>(params_->getDouble("decay", layer.decay));
  {
    std::lock_guard<std::mutex> lock(context_.map->mutex);
    index_ = context_.map->map->addSource(name_, layer);
  }
  configure(*params_);
  const std::string topic = params_->getString("topic", "");
  if (topic.empty()) {
    throw std::invalid_argument(params_->prefix() + "topic is missing");
  }
  subscribe(topic);
}

void GridSource::defer(const std::string & frame, const rclcpp::Time & stamp, std::function<void()> work)
{
  if (pending_) {
    skip();  // the previous one never got its TF
  }
  pending_ = std::move(work);
  pending_frame_ = frame;
  pending_stamp_ = stamp;
  retry();
}

void GridSource::retry()
{
  if (!pending_) {
    return;
  }
  if (tooOld(pending_stamp_)) {
    skip();
    pending_ = nullptr;
    RCLCPP_WARN_THROTTLE(
      context_.node->get_logger(), *context_.node->get_clock(), 5000, "%s: no %s -> %s in time, messages dropped",
      name_.c_str(), context_.grid_frame.c_str(), pending_frame_.c_str());
    return;
  }
  if (!context_.tf->canTransform(context_.grid_frame, pending_frame_, pending_stamp_)) {
    return;  // not yet: next tick
  }
  auto work = std::move(pending_);
  pending_ = nullptr;
  work();
}

std::optional<Eigen::Isometry3f> GridSource::sensorInBase(const std::string & frame, const rclcpp::Time &)
{
  if (mount_) {
    return mount_;
  }
  try {  // mounts are static: the latest will do
    return toIsometry(context_.tf->lookupTransform(context_.base_frame, frame, tf2::TimePointZero));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      context_.node->get_logger(), *context_.node->get_clock(), 5000, "%s: no mount for '%s': %s",
      name_.c_str(), frame.c_str(), e.what());
    return std::nullopt;
  }
}

std::optional<Eigen::Isometry3f> GridSource::baseInGrid(const rclcpp::Time & stamp)
{
  return frameInGrid(context_.base_frame, stamp);
}

std::optional<Eigen::Isometry3f> GridSource::frameInGrid(const std::string & frame, const rclcpp::Time & stamp)
{
  try {
    return toIsometry(context_.tf->lookupTransform(context_.grid_frame, frame, stamp));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      context_.node->get_logger(), *context_.node->get_clock(), 5000, "%s: no %s -> %s: %s", name_.c_str(),
      context_.grid_frame.c_str(), frame.c_str(), e.what());
    return std::nullopt;
  }
}

bool GridSource::tooOld(const rclcpp::Time & stamp) const
{
  return (context_.node->now() - stamp).seconds() > max_age_;
}

void GridSource::record(double milliseconds)
{
  std::lock_guard<std::mutex> lock(timing_mutex_);
  timing_.last_ms = milliseconds;
  timing_.max_ms = std::max(timing_.max_ms, milliseconds);
  timing_.sum_ms += milliseconds;
  ++timing_.count;
}

void GridSource::prepareRays(const MapRepresentation & map, Scan & scan, double time) const
{
  const bool limit = std::isfinite(max_mark_range_);
  if (ground_margin_ < 0.0 && !limit) {
    return;
  }
  float ground = 0.0f;
  for (Ray & ray : scan.rays) {
    if (!ray.hit) {
      continue;
    }
    if (ground_margin_ >= 0.0 &&
      map.groundNear(ray.end.x(), ray.end.y(), ground_search_radius_, time, ground_max_age_, ground) &&
      ray.end.z() - ground < ground_margin_)
    {
      ray.hit = false;  // the ground: free up to and at it
    } else if (limit && std::hypot(ray.end.x() - scan.origin.x(), ray.end.y() - scan.origin.y()) > max_mark_range_) {
      ray.mark = false;  // too far to trust: free before it, its end left alone
    }
  }
}

void GridSource::skip()
{
  std::lock_guard<std::mutex> lock(timing_mutex_);
  ++timing_.skipped;
}

// ---------------------------------------------------------------- point cloud
void PointCloudSource::configure(const RosParams & params)
{
  ground_from_map_ = params.getBool("ground_from_map", ground_from_map_);
  if (ground_from_map_ && ground_margin_ < 0.0) {
    ground_margin_ = 0.2;
  }
  clear_height_ = static_cast<float>(params.getDouble("clear_height", clear_height_));
  max_clear_range_ = static_cast<float>(params.getDouble("max_clear_range", max_clear_range_));
  clear_ = params.getBool("clear", clear_);
  deskew_ = params.getBool("deskew", deskew_);
  scan_period_ = params.getDouble("scan_period", scan_period_);
  if (params.getBool("debug_cloud", false)) {
    debug_publisher_ = context_.node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/" + name_ + "/labelled", rclcpp::SensorDataQoS());
  }
  for (const std::string & filter : params.getStrings("filters", {})) {
    const RosParams filter_params(context_.node, params.prefix() + filter + ".");
    const std::string type = filter_params.getString("type", filter);
    auto instance = context_.filters->createSharedInstance(type);
    instance->setVehicle(context_.vehicle);
    instance->initialize(filter_params);
    filters_.push_back(instance);
    filter_names_.push_back(filter);
  }
}

void PointCloudSource::subscribe(const std::string & topic)
{
  subscription_ = context_.node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m) {
      bool timed = false;
      for (const auto & field : m->fields) {
        timed |= field.name == "time" && field.datatype == sensor_msgs::msg::PointField::FLOAT32;
      }
      // A deskewed scan needs the car's pose until its last point
      const rclcpp::Time until = rclcpp::Time(m->header.stamp) +
        rclcpp::Duration::from_seconds(deskew_ && timed ? scan_period_ : 0.0);
      defer(m->header.frame_id, until, [this, m]() { onCloud(m); });
    });
}

void PointCloudSource::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message)
{
  const rclcpp::Time stamp(message->header.stamp);
  if (tooOld(stamp)) {
    skip();
    return;
  }
  const auto start = Clock::now();
  const auto sensor = sensorInBase(message->header.frame_id, stamp);
  const auto base = baseInGrid(stamp);
  if (!sensor || !base) {
    skip();
    return;
  }

  // To base_footprint
  Cloud cloud;
  cloud.origin = sensor->translation();
  const std::size_t n = static_cast<std::size_t>(message->width) * message->height;
  cloud.resize(n);
  bool has_intensity = false;
  bool has_time = false;
  for (const auto & field : message->fields) {
    has_intensity |= field.name == "intensity" && field.datatype == sensor_msgs::msg::PointField::FLOAT32;
    has_time |= field.name == "time" && field.datatype == sensor_msgs::msg::PointField::FLOAT32;
  }
  // Deskewing: base_footprint at the stamp <- base_footprint at a point's time, per 5 ms slice
  const bool deskew = deskew_ && has_time;
  constexpr double kSlice = 0.005;
  std::vector<std::optional<Eigen::Isometry3f>> slices;
  const Eigen::Isometry3f base_inverse = base->inverse();
  auto motion = [&](float time) -> const std::optional<Eigen::Isometry3f> & {
    const std::size_t k = static_cast<std::size_t>(std::clamp(time, 0.0f, 1.0f) / kSlice);
    if (k >= slices.size()) {
      slices.resize(k + 1);
    }
    if (!slices[k]) {
      const auto at = baseInGrid(stamp + rclcpp::Duration::from_seconds((k + 0.5) * kSlice));
      slices[k] = at ? base_inverse * *at : Eigen::Isometry3f::Identity();
    }
    return slices[k];
  };
  std::optional<sensor_msgs::PointCloud2ConstIterator<float>> point_time;
  if (deskew) {
    point_time.emplace(*message, "time");
  }
  std::vector<float> intensity;
  if (has_intensity) {
    intensity.resize(n);
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x"), y(*message, "y"), z(*message, "z");
  std::optional<sensor_msgs::PointCloud2ConstIterator<float>> i;
  if (has_intensity) {
    i.emplace(*message, "intensity");
  }
  std::size_t count = 0;
  for (std::size_t k = 0; k < n; ++k, ++x, ++y, ++z) {
    if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
      if (i) {
        intensity[count] = **i;
      }
      const Eigen::Vector3f p = *sensor * Eigen::Vector3f(*x, *y, *z);
      cloud.points[count++] = deskew ? Eigen::Vector3f(*motion(**point_time) * p) : p;
    }
    if (i) {
      ++*i;
    }
    if (point_time) {
      ++*point_time;
    }
  }
  cloud.resize(count);  // resize() resets the labels; the points stay
  if (has_intensity) {
    intensity.resize(count);
    cloud.intensity = std::move(intensity);
  }
  process(cloud, message->header, *base, stamp, start);
}

void PointCloudSource::groundFromMap(Cloud & cloud, const Eigen::Isometry3f & base, double time) const
{
  std::lock_guard<std::mutex> lock(context_.map->mutex);
  const MapRepresentation & map = *context_.map->map;
  for (std::size_t k = 0; k < cloud.size(); ++k) {
    if (cloud.labels[k] == Cloud::kDropped) {
      continue;
    }
    const Eigen::Vector3f p = base * cloud.points[k];
    float ground = 0.0f;
    if (map.groundNear(p.x(), p.y(), ground_search_radius_, time, ground_max_age_, ground)) {
      const float height = p.z() - ground;
      cloud.ground_z[k] = cloud.points[k].z() - height;  // the same height over it in base_footprint
      if (height < ground_margin_) {
        cloud.labels[k] = Cloud::kGround;
      }
    } else {
      cloud.labels[k] = Cloud::kUnmarked;
    }
  }
}

void PointCloudSource::process(
  Cloud & cloud, const std_msgs::msg::Header & header, const Eigen::Isometry3f & base_transform,
  const rclcpp::Time & stamp, std::chrono::steady_clock::time_point start)
{
  const Eigen::Isometry3f * base = &base_transform;
  const std::size_t count = cloud.size();
  if (ground_from_map_) {
    groundFromMap(cloud, *base, stamp.seconds());
  }
  diagnostics_.clear();
  for (std::size_t f = 0; f < filters_.size(); ++f) {
    filters_[f]->apply(cloud);
    for (const auto & [key, value] : filters_[f]->diagnostics()) {
      diagnostics_.emplace_back(filter_names_[f] + "." + key, value);
    }
  }
  if (debug_publisher_) {
    std_msgs::msg::Header debug_header = header;
    debug_header.frame_id = context_.base_frame;
    publishDebug(cloud, debug_header);
  }

  // Rays in the grid frame
  Scan scan;
  scan.origin = *base * cloud.origin;
  scan.clear_height = clear_height_;
  scan.max_clear_range = clear_ ? max_clear_range_ : 0.0f;
  scan.time = stamp.seconds();
  scan.rays.reserve(count);
  for (std::size_t k = 0; k < count; ++k) {
    if (cloud.labels[k] == Cloud::kDropped) {
      continue;
    }
    Ray ray;
    ray.end = *base * cloud.points[k];
    ray.hit = cloud.labels[k] == Cloud::kObstacle || cloud.labels[k] == Cloud::kUnmarked;
    ray.mark = cloud.labels[k] != Cloud::kUnmarked;
    if (!ray.hit && !clear_) {
      continue;
    }
    const Eigen::Vector3f & p = cloud.points[k];
    const float ground = std::isnan(cloud.ground_z[k]) ? 0.0f : cloud.ground_z[k];
    ray.ground_z = (*base * Eigen::Vector3f(p.x(), p.y(), ground)).z();
    scan.rays.push_back(ray);
  }
  const double time = scan.time;
  {
    std::lock_guard<std::mutex> lock(context_.map->mutex);
    MapRepresentation & map = *context_.map->map;
    if (provides_ground_) {
      for (std::size_t k = 0; k < count; ++k) {
        if (cloud.labels[k] == Cloud::kGround) {
          const Eigen::Vector3f p = *base * cloud.points[k];
          map.setGround(p.x(), p.y(), p.z(), time);
        }
      }
    }
    prepareRays(map, scan, time);
    map.insert(index_, scan);
  }
  record(millisecondsSince(start));
}

void PointCloudSource::publishDebug(const Cloud & cloud, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = header;
  sensor_msgs::PointCloud2Modifier modifier(out);
  modifier.setPointCloud2Fields(
    5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1,
    sensor_msgs::msg::PointField::FLOAT32, "label", 1, sensor_msgs::msg::PointField::FLOAT32, "height", 1,
    sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(cloud.size());
  sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z"), label(out, "label"),
  height(out, "height");
  for (std::size_t k = 0; k < cloud.size(); ++k, ++x, ++y, ++z, ++label, ++height) {
    const Eigen::Vector3f & p = cloud.points[k];
    *x = p.x();
    *y = p.y();
    *z = p.z();
    *label = cloud.labels[k];
    *height = std::isnan(cloud.ground_z[k]) ? std::numeric_limits<float>::quiet_NaN() : p.z() - cloud.ground_z[k];
  }
  debug_publisher_->publish(out);
}

// ---------------------------------------------------------------- depth image
void DepthImageSource::configure(const RosParams & params)
{
  PointCloudSource::configure(params);
  stride_ = std::max(1, static_cast<int>(params.getDouble("stride", stride_)));
  min_depth_ = static_cast<float>(params.getDouble("min_depth", min_depth_));
  max_depth_ = static_cast<float>(params.getDouble("max_depth", max_depth_));
  info_topic_ = params.getString("camera_info", "");
}

void DepthImageSource::subscribe(const std::string & topic)
{
  std::string info = info_topic_;
  if (info.empty()) {
    const std::size_t slash = topic.rfind('/');
    info = (slash == std::string::npos ? std::string() : topic.substr(0, slash + 1)) + "camera_info";
  }
  info_subscription_ = context_.node->create_subscription<sensor_msgs::msg::CameraInfo>(
    info, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & m) {
      std::lock_guard<std::mutex> lock(info_mutex_);
      info_ = m;
    });
  image_subscription_ = context_.node->create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Image::ConstSharedPtr & m) {
      defer(m->header.frame_id, rclcpp::Time(m->header.stamp), [this, m]() { onImage(m); });
    });
}

void DepthImageSource::onImage(const sensor_msgs::msg::Image::ConstSharedPtr & message)
{
  const rclcpp::Time stamp(message->header.stamp);
  if (tooOld(stamp)) {
    skip();
    return;
  }
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  {
    std::lock_guard<std::mutex> lock(info_mutex_);
    info = info_;
  }
  const bool metres = message->encoding == "32FC1";
  const bool millimetres = message->encoding == "16UC1" || message->encoding == "mono16";
  if (!info || info->k[0] <= 0.0 || info->k[4] <= 0.0 || (!metres && !millimetres)) {
    RCLCPP_WARN_THROTTLE(
      context_.node->get_logger(), *context_.node->get_clock(), 5000,
      "%s: no camera_info yet, or a depth encoding other than 32FC1/16UC1 ('%s')", name_.c_str(),
      message->encoding.c_str());
    skip();
    return;
  }
  const auto start = Clock::now();
  const auto sensor = sensorInBase(message->header.frame_id, stamp);
  const auto base = baseInGrid(stamp);
  if (!sensor || !base) {
    skip();
    return;
  }
  // Back-projection: the optical frame (z forward, x right, y down)
  const float fx = static_cast<float>(info->k[0]);
  const float fy = static_cast<float>(info->k[4]);
  const float cx = static_cast<float>(info->k[2]);
  const float cy = static_cast<float>(info->k[5]);
  const int w = static_cast<int>(message->width);
  const int h = static_cast<int>(message->height);
  Cloud cloud;
  cloud.origin = sensor->translation();
  cloud.points.reserve(static_cast<std::size_t>((w / stride_ + 1) * (h / stride_ + 1)));
  for (int v = stride_ / 2; v < h; v += stride_) {
    const uint8_t * row = message->data.data() + static_cast<std::size_t>(v) * message->step;
    for (int u = stride_ / 2; u < w; u += stride_) {
      float d = 0.0f;
      if (metres) {
        std::memcpy(&d, row + 4 * u, 4);
      } else {
        uint16_t mm = 0;
        std::memcpy(&mm, row + 2 * u, 2);
        d = mm * 0.001f;
      }
      if (!std::isfinite(d) || d < min_depth_ || d > max_depth_) {
        continue;  // no return, or beyond what the depth is good for
      }
      cloud.points.push_back(*sensor * Eigen::Vector3f((u - cx) * d / fx, (v - cy) * d / fy, d));
    }
  }
  const std::size_t n = cloud.points.size();
  cloud.labels.assign(n, Cloud::kObstacle);
  cloud.ground_z.assign(n, std::numeric_limits<float>::quiet_NaN());
  process(cloud, message->header, *base, stamp, start);
}

// ---------------------------------------------------------------- laser scan
void LaserScanSource::configure(const RosParams & params)
{
  max_clear_range_ = static_cast<float>(params.getDouble("max_clear_range", max_clear_range_));
  clear_max_range_ = params.getBool("clear_max_range", clear_max_range_);
}

void LaserScanSource::subscribe(const std::string & topic)
{
  subscription_ = context_.node->create_subscription<sensor_msgs::msg::LaserScan>(
    topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr & m) {
      defer(m->header.frame_id, rclcpp::Time(m->header.stamp), [this, m]() { onScan(m); });
    });
}

void LaserScanSource::onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & message)
{
  const rclcpp::Time stamp(message->header.stamp);
  if (tooOld(stamp)) {
    skip();
    return;
  }
  const auto start = Clock::now();
  const auto sensor = sensorInBase(message->header.frame_id, stamp);
  const auto base = baseInGrid(stamp);
  if (!sensor || !base) {
    skip();
    return;
  }
  const Eigen::Isometry3f to_grid = *base * *sensor;
  Scan scan;
  scan.origin = to_grid.translation();
  scan.max_clear_range = max_clear_range_;
  scan.time = stamp.seconds();
  scan.rays.reserve(message->ranges.size());
  for (std::size_t k = 0; k < message->ranges.size(); ++k) {
    float range = message->ranges[k];
    Ray ray;
    if (std::isfinite(range) && range >= message->range_min && range <= message->range_max) {
      ray.hit = true;
    } else if (clear_max_range_ && !std::isnan(range) && range > message->range_min) {
      ray.hit = false;
      range = std::min(max_clear_range_, message->range_max);
    } else {
      continue;
    }
    const float angle = message->angle_min + static_cast<float>(k) * message->angle_increment;
    ray.end = to_grid * Eigen::Vector3f(range * std::cos(angle), range * std::sin(angle), 0.0f);
    scan.rays.push_back(ray);  // ground_z -inf: free all along
  }
  {
    std::lock_guard<std::mutex> lock(context_.map->mutex);
    prepareRays(*context_.map->map, scan, scan.time);
    context_.map->map->insert(index_, scan);
  }
  record(millisecondsSince(start));
}

// ---------------------------------------------------------------- occupancy grid
void OccupancyGridSource::configure(const RosParams & params)
{
  occupied_threshold_ = static_cast<int>(params.getDouble("occupied_threshold", occupied_threshold_));
  free_threshold_ = static_cast<int>(params.getDouble("free_threshold", free_threshold_));
}

void OccupancyGridSource::subscribe(const std::string & topic)
{
  subscription_ = context_.node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::QoS(1),
    [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & m) {
      defer(m->header.frame_id, rclcpp::Time(m->header.stamp), [this, m]() { onGrid(m); });
    });
}

void OccupancyGridSource::onGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & message)
{
  const rclcpp::Time stamp(message->header.stamp);
  if (tooOld(stamp)) {
    skip();
    return;
  }
  const auto start = Clock::now();
  const auto frame = frameInGrid(message->header.frame_id, stamp);
  if (!frame) {
    skip();
    return;
  }
  const auto & info = message->info;
  Eigen::Isometry3f origin = Eigen::Isometry3f::Identity();
  origin.translation() = Eigen::Vector3f(info.origin.position.x, info.origin.position.y, info.origin.position.z);
  origin.linear() = Eigen::Quaternionf(
    info.origin.orientation.w, info.origin.orientation.x, info.origin.orientation.y,
    info.origin.orientation.z).toRotationMatrix();
  const Eigen::Isometry3f to_grid = *frame * origin;
  const float res = info.resolution;
  {
    std::lock_guard<std::mutex> lock(context_.map->mutex);
    MapRepresentation & map = *context_.map->map;
    for (unsigned j = 0; j < info.height; ++j) {
      for (unsigned i = 0; i < info.width; ++i) {
        const int8_t v = message->data[j * info.width + i];
        if (v < 0 || (v > free_threshold_ && v < occupied_threshold_)) {
          continue;
        }
        const Eigen::Vector3f p = to_grid * Eigen::Vector3f((i + 0.5f) * res, (j + 0.5f) * res, 0.0f);
        map.set(index_, p.x(), p.y(), v >= occupied_threshold_ ? 1e3f : -1e3f);  // clamped to the layer's range
      }
    }
  }
  record(millisecondsSince(start));
}

}  // namespace sac_perception

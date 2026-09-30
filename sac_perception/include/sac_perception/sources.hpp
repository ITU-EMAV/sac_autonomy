// Grid sources: one per sensor in the config, a plugin per message type (pluginlib, base
// class sac_perception::GridSource). Each hands its scans to the map representation
// (map_representation.hpp), as its own source there (a layer of the 2D grid in
// direct_projection).
//
//   sources:
//     names: [roof_lidar, front_lidar]
//     roof_lidar:
//       type: pointcloud            # pointcloud | laser_scan | occupancy_grid
//       topic: /sac/sensors/roof_lidar/points
//       filters: [self, ground, band]
//       self:   {type: crop_box, ...}
//       ...                         # layer and source parameters
//
// Where a sensor sits comes from TF (the message's frame_id; the URDF's mount), or from
// `mount: [x, y, z, roll, pitch, yaw]` in base_footprint for a sensor without TF. Each
// message is placed with odom -> sensor at its own stamp, so a moving car's scans land where
// they were taken.
//
// Common parameters: topic, mount, max_age [s] (older messages are skipped), and the layer's
// hit, miss, min, max, decay (grid.hpp LayerParams).
//
// The shared ground map (grid.hpp), for sensors that complete each other:
//   provides_ground (false): a source whose filters find the ground (a point cloud with
//     ground_patchwork) writes its ground points' heights into it
//   ground_margin [m] (off): a source that cannot tell the ground from an obstacle (a 2D
//     lidar, whose scan plane meets a road rising ahead) does not mark a return that lies
//     within this of the ground there, and takes it as free space instead. Where no ground is
//     known the return stays an obstacle. Also: ground_search_radius [m] (1.0) around the
//     return, ground_max_age [s] (3.0)
// Neither names a sensor: any number may provide or use it, and without a provider
// ground_margin changes nothing.
//   max_mark_range [m] (none): returns further away are not marked as obstacles, only the
//     space before them is cleared (a 2D lidar far ahead often meets a road rising where
//     no other sensor has seen the ground yet; near the car it covers what the others miss)

#pragma once

#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/buffer.h>

#include "sac_perception/filters.hpp"
#include "sac_perception/map_representation.hpp"
#include "sac_perception/ros_params.hpp"

namespace sac_perception
{

/// The map the sources write into, shared with the node (which publishes it).
struct SharedMap
{
  std::mutex mutex;
  std::shared_ptr<MapRepresentation> map;
};

struct SourceContext
{
  rclcpp::Node * node = nullptr;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::string grid_frame;   // odom
  std::string base_frame;   // base_footprint
  SharedMap * map = nullptr;
  pluginlib::ClassLoader<PointFilter> * filters = nullptr;
  VehicleBox vehicle;       // the car's box, for the filters (may be unknown)
};

/// Processing time of the last messages, for ~/timing.
struct SourceTiming
{
  double last_ms = 0.0;
  double max_ms = 0.0;
  double sum_ms = 0.0;
  std::uint64_t count = 0;
  std::uint64_t skipped = 0;  // no TF, too old
};

class GridSource
{
public:
  virtual ~GridSource() = default;

  /// Reads "sources.<name>.", adds itself to the map and subscribes.
  void initialize(const SourceContext & context, const std::string & name, std::shared_ptr<RosParams> params);

  const std::string & name() const { return name_; }
  SourceTiming timing() const
  {
    std::lock_guard<std::mutex> lock(timing_mutex_);
    return timing_;
  }

  /// Numbers about the last message for tuning (e.g. a ground filter's), with the timing
  virtual std::vector<std::pair<std::string, double>> diagnostics() const { return {}; }

  /// Processes the message waiting for its TF, if the TF is there now (the node calls this
  /// every tick; no source ever blocks waiting for a transform).
  void retry();

protected:
  virtual void configure(const RosParams & params) { (void)params; }
  virtual void subscribe(const std::string & topic) = 0;

  /// Processes a message now if odom -> `frame` at `stamp` is known, else keeps it for
  /// retry() (a newer message replaces it; one too old is dropped)
  void defer(const std::string & frame, const rclcpp::Time & stamp, std::function<void()> work);

  /// base_footprint <- sensor (TF at the stamp, static in practice, or `mount`)
  std::optional<Eigen::Isometry3f> sensorInBase(const std::string & frame, const rclcpp::Time & stamp);
  /// grid frame (odom) <- base_footprint at the stamp
  std::optional<Eigen::Isometry3f> baseInGrid(const rclcpp::Time & stamp);
  /// grid frame <- any frame at the stamp
  std::optional<Eigen::Isometry3f> frameInGrid(const std::string & frame, const rclcpp::Time & stamp);
  bool tooOld(const rclcpp::Time & stamp) const;
  void record(double milliseconds);
  void skip();
  /// Before a scan goes into the grid (grid locked): with ground_margin, returns lying on the
  /// known ground become free rays; returns beyond max_mark_range are not marked
  void prepareRays(const MapRepresentation & map, Scan & scan, double time) const;

  bool provides_ground_ = false;
  double ground_margin_ = -1.0;  // < 0: off
  double ground_search_radius_ = 1.0;
  double ground_max_age_ = 3.0;
  double max_mark_range_ = std::numeric_limits<double>::infinity();

  SourceContext context_;
  std::string name_;
  std::shared_ptr<RosParams> params_;
  int index_ = 0;  // in the map
  double max_age_ = 0.5;
  std::optional<Eigen::Isometry3f> mount_;

private:
  mutable std::mutex timing_mutex_;
  SourceTiming timing_;
  std::function<void()> pending_;
  std::string pending_frame_;
  rclcpp::Time pending_stamp_;
};

/// sensor_msgs/PointCloud2 (3D lidar, depth camera): x, y, z to base_footprint, the filter
/// chain, then obstacle points mark their cells and ground points clear the space before
/// them (where the ray runs lower than `clear_height` over the ground).
///   filters: names of the filters in order, each with its parameters under its name
///   clear_height [m] (0.3), max_clear_range [m] (40), clear (true)
///   debug_cloud (false): publish the filtered points on ~/<name>/labelled (base_footprint;
///     fields x, y, z, label 0 obstacle / 1 ground / 2 dropped, height over the ground)
class PointCloudSource : public GridSource
{
protected:
  void configure(const RosParams & params) override;
  void subscribe(const std::string & topic) override;

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message);
  void publishDebug(const Cloud & cloud, const std_msgs::msg::Header & header);
  std::vector<std::pair<std::string, double>> diagnostics() const override { return diagnostics_; }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_publisher_;
  std::vector<std::shared_ptr<PointFilter>> filters_;
  std::vector<std::string> filter_names_;
  std::vector<std::pair<std::string, double>> diagnostics_;
  float clear_height_ = 0.3f;
  float max_clear_range_ = 40.0f;
  bool clear_ = true;
};

/// sensor_msgs/LaserScan (2D lidar): returns mark their cells, the space before them is free
/// (the scan plane is at the sensor's height all along).
///   max_clear_range [m] (40); clear_max_range: no return = free up to max_clear_range (false)
class LaserScanSource : public GridSource
{
protected:
  void configure(const RosParams & params) override;
  void subscribe(const std::string & topic) override;

private:
  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & message);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  float max_clear_range_ = 40.0f;
  bool clear_max_range_ = false;
};

/// nav_msgs/OccupancyGrid from another node (e.g. a camera's segmentation), in any frame:
/// its cells overwrite this layer (occupied >= occupied_threshold, free <= free_threshold,
/// unknown left as it is).
class OccupancyGridSource : public GridSource
{
protected:
  void configure(const RosParams & params) override;
  void subscribe(const std::string & topic) override;

private:
  void onGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & message);
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  int occupied_threshold_ = 65;
  int free_threshold_ = 25;
};

}  // namespace sac_perception

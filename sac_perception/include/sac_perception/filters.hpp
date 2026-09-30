// Point filters: a chain per point cloud source, set in its YAML, in the order given
//   sources.roof_lidar.filters: [self, voxel, ground, band]
//   sources.roof_lidar.self: {type: crop_box, ...}      # each filter's type and parameters
// Plugins (pluginlib, base class sac_perception::PointFilter); the built-in ones are
// crop_box, voxel, ground_height, ground_patchwork (ground_patchwork.hpp), height_band.
// A new method is a new plugin and a name in the list. Each source has its own instances,
// so a filter that learns from the scans (ground_patchwork) learns per sensor. They work on
// the points in base_footprint (x forward, z up), before they go into the grid. No ROS here.

#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "sac_perception/params.hpp"
#include "sac_perception/vehicle.hpp"

namespace sac_perception
{

/// A point cloud on its way through the filters, in base_footprint.
struct Cloud
{
  enum Label : uint8_t { kObstacle = 0, kGround = 1, kDropped = 2 };

  Eigen::Vector3f origin{0.0f, 0.0f, 0.0f};  // the sensor
  std::vector<Eigen::Vector3f> points;
  std::vector<uint8_t> labels;                // all kObstacle to start with
  std::vector<float> ground_z;                // height of the ground under each point (NaN: unknown)
  std::vector<float> intensity;               // per point, empty when the sensor gives none

  void resize(std::size_t n)
  {
    points.resize(n);
    labels.assign(n, kObstacle);
    ground_z.assign(n, std::numeric_limits<float>::quiet_NaN());
  }
  std::size_t size() const { return points.size(); }
};

class PointFilter
{
public:
  virtual ~PointFilter() = default;
  /// The car's box (from the URDF), given before initialize(); may be unknown
  void setVehicle(const VehicleBox & vehicle) { vehicle_ = vehicle; }
  virtual void initialize(const Params & params) = 0;
  /// Not const: a filter may learn from the scans it sees (one instance per source)
  virtual void apply(Cloud & cloud) = 0;
  /// Numbers about the last scan, for tuning (published with the source's timing)
  virtual std::vector<std::pair<std::string, double>> diagnostics() const { return {}; }

protected:
  VehicleBox vehicle_;
};

/// Drops the points inside a box (the car itself: body, rack, sensor posts), or outside it
/// with `keep_inside`. min, max: [x, y, z] in base_footprint; or `from: vehicle`: the car's
/// box from the URDF grown by `margin` [m] (0.1) on every side. max_range [m]: also drops
/// the points further than this from the sensor.
class CropBox : public PointFilter
{
public:
  void initialize(const Params & params) override;
  void apply(Cloud & cloud) override;

private:
  Eigen::Vector3f min_{-1.6f, -0.95f, -0.5f};
  Eigen::Vector3f max_{1.7f, 0.95f, 2.3f};
  bool keep_inside_ = false;
  float margin_ = 0.1f;
  float max_range_ = std::numeric_limits<float>::infinity();  // also drop points further away
};

/// Ground: points lower than `height` [m] over base_footprint's plane. Only for flat ground.
class GroundHeight : public PointFilter
{
public:
  void initialize(const Params & params) override;
  void apply(Cloud & cloud) override;

private:
  float height_ = 0.2f;
};


/// Keeps the obstacle points between `min` and the top of the band [m] over the ground under
/// them (over base_footprint's plane where the ground is unknown): points lower than `min`
/// are drivable (kerbs, bumps: ground), points over the top are overhead (branches, signs,
/// bridges: dropped). The bottom follows the ground found by the filters before; the top:
///   max_from: fixed     `max` [m] over the ground (2.5)
///             vehicle   the car's height (its box from the URDF) + `clearance` [m] (0.3): what
///                       the car would touch; nothing to set for another car or sensor
///             sensor    the sensor's height over base_footprint + `clearance`
class HeightBand : public PointFilter
{
public:
  void initialize(const Params & params) override;
  void apply(Cloud & cloud) override;

private:
  float min_ = 0.2f;
  float max_ = 2.5f;
  enum class Top { kFixed, kVehicle, kSensor } top_ = Top::kFixed;
  float clearance_ = 0.3f;
};

/// Keeps one point per `size` [m] voxel (the first), fewer rays to trace.
class Voxel : public PointFilter
{
public:
  void initialize(const Params & params) override;
  void apply(Cloud & cloud) override;

private:
  float size_ = 0.1f;
};

}  // namespace sac_perception

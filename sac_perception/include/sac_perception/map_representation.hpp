// How the node keeps what the sensors saw, and turns it into the 2D grid the planner reads.
// A plugin (pluginlib, base class sac_perception::MapRepresentation), chosen in the YAML:
//
//   map:
//     type: direct_projection     # direct_projection | (sparse_voxel | multi_level_surface)
//     ...                         # its parameters
//
// The sources hand every representation the same thing: a scan of rays in the grid frame
// (grid.hpp Scan), each ending in an obstacle point or a free one (the ground), with the ground
// height under it. What a representation does with them is its own: direct_projection writes
// each source's rays straight into a 2D layer (the sources' filters have already decided
// what is an obstacle); a 3D one keeps the points and decides per column.
// All of them publish the same window: a square in the grid frame that follows the car by
// whole cells (grid.size, grid.resolution, grid.recenter_distance). No ROS here.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "sac_perception/grid.hpp"
#include "sac_perception/params.hpp"
#include "sac_perception/vehicle.hpp"

namespace sac_perception
{

/// The published window
struct GridGeometry
{
  double size = 80.0;               // [m] square
  double resolution = 0.2;          // [m]
  double recenter_distance = 2.0;   // [m]
};

class MapRepresentation
{
public:
  virtual ~MapRepresentation() = default;

  /// `params` under "map."; `vehicle` from the URDF (may be unknown)
  virtual void initialize(const Params & params, const GridGeometry & geometry, const VehicleBox & vehicle) = 0;

  /// A source's own share of the map (a layer, its own hit/miss/decay); returns its index
  virtual int addSource(const std::string & name, const LayerParams & params) = 0;

  /// Keeps (x, y) near the window's centre
  virtual void recenter(double x, double y) = 0;
  /// What has not been seen for a while fades, over dt [s]
  virtual void decay(double dt) = 0;

  /// A scan of a source, in the grid frame
  virtual void insert(int source, const Scan & scan) = 0;
  /// A cell of a source's from another node's 2D grid (log-odds, clamped by the source)
  virtual void set(int source, double x, double y, float log_odds) = 0;

  /// The shared ground height map (grid.hpp RollingGrid::setGround, groundNear)
  virtual void setGround(double x, double y, float z, double time) = 0;
  virtual bool groundNear(
    double x, double y, double radius, double time, double max_age, float & z) const = 0;

  /// The window
  virtual int width() const = 0;
  virtual double resolution() const = 0;
  virtual double originX() const = 0;
  virtual double originY() const = 0;
  /// Where the car cannot go, as nav_msgs/OccupancyGrid data (row-major from the origin
  /// corner): -1 unknown, 0..100 occupancy probability
  virtual std::vector<int8_t> project() const = 0;

  /// Numbers for tuning, published with the timing
  virtual std::vector<std::pair<std::string, double>> diagnostics() const { return {}; }
};

/// The grid of Stage 1, as it was: each source's filters decide per point what is an
/// obstacle (ground_patchwork, height_band), each source writes its own 2D layer (marks
/// the obstacles, clears the space before them), and the published grid is the most
/// occupied layer per cell. The fastest; knows one ground height per cell, so a bridge
/// relies on the ground filter (ground_patchwork's grade check) and height_band.
/// Parameters: none of its own.
class DirectProjection : public MapRepresentation
{
public:
  void initialize(const Params & params, const GridGeometry & geometry, const VehicleBox & vehicle) override;
  int addSource(const std::string & name, const LayerParams & params) override
  {
    return grid_->addLayer(name, params);
  }
  void recenter(double x, double y) override { grid_->recenter(x, y); }
  void decay(double dt) override { grid_->decay(dt); }
  void insert(int source, const Scan & scan) override { grid_->integrate(source, scan); }
  void set(int source, double x, double y, float log_odds) override { grid_->set(source, x, y, log_odds); }
  void setGround(double x, double y, float z, double time) override { grid_->setGround(x, y, z, time); }
  bool groundNear(double x, double y, double radius, double time, double max_age, float & z) const override
  {
    return grid_->groundNear(x, y, radius, time, max_age, z);
  }
  int width() const override { return grid_->width(); }
  double resolution() const override { return grid_->resolution(); }
  double originX() const override { return grid_->originX(); }
  double originY() const override { return grid_->originY(); }
  std::vector<int8_t> project() const override { return grid_->combined(); }

  const RollingGrid & grid() const { return *grid_; }

private:
  std::unique_ptr<RollingGrid> grid_;
};

}  // namespace sac_perception

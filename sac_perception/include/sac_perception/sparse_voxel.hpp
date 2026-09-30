// sparse_voxel: the sensors' obstacle points kept in 3D, in voxels of the grid's cell size
// and `voxel_height`, and projected to the 2D grid by the traversability rules
// (traversability.hpp). All sources share one 3D space, so what one sensor saw at 1 m is not
// cleared by another's ray passing at 1.5 m over it, and the ground and what is over it are
// held per column over time (a bridge's deck and railings are over the car; a ceiling
// lower than it closes the way).
//
// Sparse in height, dense over the ground: each cell of the rolling window holds the few
// occupied voxels of its column. A ray is traced in 2D over the cells as in direct_projection
// and lowers the voxels of each column it passes through at the height it runs there, so
// clearing costs no more than in 2D (most columns are empty); a free voxel is not kept.
// Each voxel keeps the lowest and highest point that hit it, so the band's edges are exact
// and not rounded to voxels.
//
// Two modes (map.memory):
//   true   (default) what was seen stays until rays pass through it or it fades: an obstacle
//          out of a sensor's sight (the blind zone around the car, between a 16-beam lidar's
//          rings) is kept; a moving one leaves a short trail
//   false  each source's voxels are its scans of the last `window` [s] (0: its last scan)
//          only, rebuilt with every scan, no rays traced: cheaper, no trails behind moving
//          things, but what a sensor does not see now is gone
// The ground map keeps its memory in both (the 2D lidar leans on it).
//
// Parameters (map.): memory (true), window [s] (0), voxel_height [m] (0.2), and the
// traversability rules (min_obstacle_height,
// max_from, clearance, max, max_step, ground_search_radius, ground_max_age, level_gap).
// Each source keeps its own hit, miss, clamps and decay (LayerParams) for the voxels it saw
// last. A 2D grid from another node (occupancy_grid source) stays a 2D layer, combined with
// the projection.

#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "sac_perception/map_representation.hpp"
#include "sac_perception/traversability.hpp"

namespace sac_perception
{

class SparseVoxel : public MapRepresentation
{
public:
  void initialize(const Params & params, const GridGeometry & geometry, const VehicleBox & vehicle) override;
  int addSource(const std::string & name, const LayerParams & params) override;
  void recenter(double x, double y) override;
  void decay(double dt) override;
  void insert(int source, const Scan & scan) override;
  void set(int source, double x, double y, float log_odds) override;
  void setGround(double x, double y, float z, double time) override { plane_->setGround(x, y, z, time); }
  bool groundNear(double x, double y, double radius, double time, double max_age, float & z) const override
  {
    return plane_->groundNear(x, y, radius, time, max_age, z);
  }
  int width() const override { return plane_->width(); }
  double resolution() const override { return plane_->resolution(); }
  double originX() const override { return plane_->originX(); }
  double originY() const override { return plane_->originY(); }
  std::vector<int8_t> project() const override;
  void points(std::vector<MapPoint> & out) const override;
  std::vector<std::pair<std::string, double>> diagnostics() const override;

  /// For tests: the column's ground as the projection sees it (NaN: none)
  float columnGround(double x, double y) const;

private:
  struct Voxel
  {
    int16_t iz;          // floor(z / voxel_height)
    uint8_t source;      // who saw it last (its decay)
    uint8_t unused = 0;
    float log_odds;
    float lo, hi;        // [m] the lowest and highest point that hit it
    uint32_t hit_scan;   // the scan that hit it last: its rays do not lower it
    float time;          // [s] since the start, when it was hit last
  };
  struct Column
  {
    std::vector<Voxel> voxels;
    float estimated_ground = std::numeric_limits<float>::quiet_NaN();  // from the ground filter
    float seen = -1.0f;   // [s] since the start: a ray passed or a point fell here
  };
  struct Source
  {
    LayerParams params;
    float hit, miss;      // log-odds steps
    int layer = -1;       // a 2D grid's own layer of plane_
  };

  /// The ground under the column k at `time` (NaN: none known)
  float ground(int k, int i, int j, double time) const;
  void hit(Column & column, const Source & source, int source_index, const Eigen::Vector3f & p, float now);

  Traversability rules_;
  float voxel_height_ = 0.2f;
  bool memory_ = true;
  float window_ = 0.0f;
  std::unique_ptr<RollingGrid> plane_;  // the window, the ground map and 2D layers
  std::vector<Column> columns_;
  std::vector<Source> sources_;
  uint32_t scan_counter_ = 0;
  double time_ = 0.0;        // the latest scan's
  double start_time_ = -1.0; // `seen` is kept as float seconds since this
  std::size_t voxel_count_ = 0;
};

}  // namespace sac_perception

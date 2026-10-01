// What the 3D representations share: the sensors' obstacle points kept per column of the
// rolling window, and projected to the 2D grid by the traversability rules
// (traversability.hpp). All sources share one 3D space, so what one sensor saw at 1 m is not
// cleared by another's ray passing at 1.5 m over it, and the ground and what is over it are
// held per column over time (a bridge's deck and railings are over the car; a ceiling
// lower than it closes the way).
//
// Dense over the ground, sparse in height: each cell of the window holds the few elements of
// its column: voxels (sparse_voxel.hpp) or height intervals (multi_level_surface.hpp); only
// how a point joins a column and what a ray passing through it clears are theirs. A ray is
// traced in 2D over the cells as in direct_projection and lowers the elements of each column
// it passes through at the height it runs there, so clearing costs about what it does in 2D
// per cell (most columns are empty); a free element is not kept. Each element keeps the
// lowest and highest point that hit it, so the band's edges are those of the points.
//
// Tracing, two shortcuts (as OctoMap's discretized insertion and Voxblox's merged
// integrator, and OpenVDB's hierarchical DDA):
//   merge_rays (true)  the rays ending in the same cell and voxel height are traced once, to
//                      their mean end (the points themselves still go in one by one): near the
//                      car dozens of ground returns share a cell. A cell along them is lowered
//                      once for them, not once per ray
//   blocks of 8 x 8 cells with no element are crossed in one step; what was seen is kept per
//   block there (it only tells free from unknown in the grid, the planner reads neither)
//
// Two modes (map.memory):
//   true   (default) what was seen stays until rays pass through it or it fades: an obstacle
//          out of a sensor's sight (the blind zone around the car, between a 16-beam lidar's
//          rings) is kept; a moving one leaves a short trail
//   false  each source's elements are its scans of the last `window` [s] (0: its last scan)
//          only, rebuilt with every scan, no rays traced: cheaper, no trails behind moving
//          things, but what a sensor does not see now is gone
// The ground map keeps its memory in both (the 2D lidar leans on it).
//
// The ground under a column with elements is looked for (its own cell, else the nearest within
// ground_search_radius: 121 cells) once per `ground_cache_time` [s] (0.5), not at every
// projection: it is held for seconds anyway (ground_max_age).
//
// Parameters (map.): memory (true), window [s] (0), merge_rays (true), ground_cache_time [s]
// (0.5), the representation's own, and the
// traversability rules (min_obstacle_height, max_from, clearance, max, max_step,
// ground_search_radius, ground_max_age, level_gap). Each source keeps its own hit, miss,
// clamps and decay (LayerParams) for the elements it saw last. A 2D grid from another node
// (occupancy_grid source) stays a 2D layer, combined with the projection.

#pragma once

#include <algorithm>
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

class ColumnMap : public MapRepresentation
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
  /// The elements in the band hit since then, at their cells' centres
  void recent(double since, std::vector<RecentPoint> & out) const override;
  double latest() const override { return time_; }
  std::vector<std::pair<std::string, double>> diagnostics() const override;

  /// For tests: the column's ground as the projection sees it (NaN: none)
  float columnGround(double x, double y) const;
  /// For tests: the elements of a column, as [lo, hi]
  std::vector<std::pair<float, float>> column(double x, double y) const;

protected:
  struct Element
  {
    float lo, hi;        // [m] the lowest and highest point that hit it
    float log_odds;
    float time;          // [s] since the start, when it was hit last
    uint32_t hit_scan;   // the scan that hit it last: its rays do not lower it
    int16_t iz;          // a voxel's index in height (sparse_voxel)
    uint8_t source;      // who saw it last (its decay)
  };
  struct Column
  {
    std::vector<Element> elements;
    float estimated_ground = std::numeric_limits<float>::quiet_NaN();  // from the ground filter
    float seen = -1.0f;  // [s] since the start: a ray passed or a point fell here
    float free = -std::numeric_limits<float>::infinity();  // [s] since the start: found free
    // The ground under it as last found, and when (found again after ground_cache_time)
    mutable float ground = std::numeric_limits<float>::quiet_NaN();
    mutable float ground_at = -std::numeric_limits<float>::infinity();
  };
  struct Source
  {
    LayerParams params;
    float hit, miss;     // log-odds steps
    int layer = -1;      // a 2D grid's own layer of plane_
  };

  /// The representation's own parameters (under map.)
  virtual void configure(const Params & params) = 0;
  /// A point joins its column
  virtual void add(Column & column, const Eigen::Vector3f & p) = 0;
  /// Does a ray running from z_min to z_max through the column pass through the element?
  virtual bool crosses(const Element & element, float z_min, float z_max) const = 0;
  /// Where to draw an element (~/map): heights
  virtual void draw(const Element & element, std::vector<float> & heights) const = 0;
  virtual const char * elementName() const = 0;

  /// For add(): raises an element for the current point (once per scan), or makes one
  void raise(Element & element, const Eigen::Vector3f & p);
  Element & make(Column & column, const Eigen::Vector3f & p);

  float element_height_ = 0.2f;  // [m] the voxels' height, and the step of drawing

private:
  /// The ground under the column k at `time` (NaN: none known), as found within the last
  /// ground_cache_time
  float ground(int k, int i, int j, double time) const;
  /// The same, looked for now: its own cell, else the nearest within ground_search_radius,
  /// else the ground filter's estimate
  float findGround(int k, int i, int j, double time) const;
  void forget(Column & column);
  /// Which blocks of cells hold an element
  void markBlocks();
  /// f(i, j, k) for each cell with elements, block by block
  template<typename F>
  void forEachFull(F && f) const
  {
    const int w = plane_->width();
    for (int bj = 0; bj < blocks_; ++bj) {
      for (int bi = 0; bi < blocks_; ++bi) {
        if (!block_full_[bj * blocks_ + bi]) {
          continue;
        }
        for (int j = bj * kBlock; j < std::min(w, (bj + 1) * kBlock); ++j) {
          for (int i = bi * kBlock; i < std::min(w, (bi + 1) * kBlock); ++i) {
            const int k = plane_->index(i, j);
            if (!columns_[k].elements.empty()) {
              f(i, j, k);
            }
          }
        }
      }
    }
  }
  static constexpr int kBlock = 8;

  Traversability rules_;
  bool memory_ = true;
  float window_ = 0.0f;
  bool merge_rays_ = true;
  float ground_cache_time_ = 0.5f;  // [s]
  int blocks_ = 0;                   // blocks across
  std::vector<uint8_t> block_full_;  // holds an element
  std::vector<float> block_seen_;    // [s] since the start: a ray crossed it
  std::vector<float> block_free_;    // [s] since the start: a ray crossed it with nothing in it
  std::vector<Eigen::Vector3f> trace_;  // the rays to trace (merged)
  std::unique_ptr<RollingGrid> plane_;  // the window, the ground map and 2D layers
  std::vector<Column> columns_;
  std::vector<Source> sources_;
  uint32_t scan_counter_ = 0;
  double time_ = 0.0;         // the latest scan's
  double start_time_ = -1.0;  // times are kept as float seconds since this
  // The point being added (for raise, make)
  const Source * source_ = nullptr;
  int source_index_ = 0;
  float now_ = 0.0f;
};

}  // namespace sac_perception

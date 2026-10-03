// multi_level_surface: each column holds a few height intervals [lo, hi] instead of voxels,
// after the multi-level surface maps of Triebel, Pfaff and Burgard (IROS 2006), which keep
// several surfaces per cell with their vertical extent (a road and a bridge over it). Here
// an interval is where obstacle points are, the ground map holding the surfaces
// (column_map.hpp for the modes, the rules and the parameters they share).
//
// A point within `merge_gap` [m] (0.3) of an interval joins it and stretches it; intervals
// that come that close merge; else it starts one of its own, up to `max_levels` (6) per
// column (then the two closest merge). A pole, a pedestrian or a car's side is one interval
// however tall, a bridge's deck another one 8 m up: fewer elements than voxels, and a ray
// lowers an interval it runs through at any height, taking each interval as `thickness` [m]
// (0.1) thicker on both sides than its points (a single point is not a line: a ray 3 cm off
// it passes through it). An interval only grows while it lasts: it is lowered as a whole,
// not cut back.

#pragma once

#include "sac_perception/column_map.hpp"

namespace sac_perception
{

class MultiLevelSurface : public ColumnMap
{
protected:
  void configure(const Params & params) override;
  void add(Column & column, const Eigen::Vector3f & p) override;
  bool crosses(const Element & element, float z_min, float z_max) const override;
  void draw(const Element & element, std::vector<float> & heights) const override;
  const char * elementName() const override { return "levels"; }

private:
  /// Merges into `into` the intervals of the column that came within merge_gap of it
  void mergeNear(Column & column, std::size_t into);

  float merge_gap_ = 0.3f;
  float thickness_ = 0.1f;
  std::size_t max_levels_ = 6;
};

}  // namespace sac_perception

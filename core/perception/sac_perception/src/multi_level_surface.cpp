#include "sac_perception/multi_level_surface.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace sac_perception
{

namespace
{
/// The gap between two intervals (negative: they overlap)
float gap(float lo_a, float hi_a, float lo_b, float hi_b) { return std::max(lo_a, lo_b) - std::min(hi_a, hi_b); }
}  // namespace

void MultiLevelSurface::configure(const Params & params)
{
  merge_gap_ = static_cast<float>(params.getDouble("merge_gap", merge_gap_));
  thickness_ = static_cast<float>(params.getDouble("thickness", thickness_));
  const double levels = params.getDouble("max_levels", static_cast<double>(max_levels_));
  if (merge_gap_ < 0.0f || levels < 1.0) {
    throw std::invalid_argument("map merge_gap must be >= 0, max_levels >= 1");
  }
  max_levels_ = static_cast<std::size_t>(levels);
  element_height_ = static_cast<float>(params.getDouble("draw_step", element_height_));
}

void MultiLevelSurface::mergeNear(Column & column, std::size_t into)
{
  auto & e = column.elements;
  for (std::size_t k = 0; k < e.size();) {
    if (k != into && gap(e[k].lo, e[k].hi, e[into].lo, e[into].hi) <= merge_gap_) {
      Element & a = e[into];
      const Element & b = e[k];
      a.lo = std::min(a.lo, b.lo);
      a.hi = std::max(a.hi, b.hi);
      if (b.log_odds > a.log_odds) {
        a.log_odds = b.log_odds;
        a.source = b.source;
      }
      a.time = std::max(a.time, b.time);
      a.hit_scan = std::max(a.hit_scan, b.hit_scan);
      e.erase(e.begin() + static_cast<std::ptrdiff_t>(k));
      if (k < into) {
        --into;
      }
      k = 0;  // it grew: look again
    } else {
      ++k;
    }
  }
}

void MultiLevelSurface::add(Column & column, const Eigen::Vector3f & p)
{
  auto & e = column.elements;
  for (std::size_t k = 0; k < e.size(); ++k) {
    if (p.z() >= e[k].lo - merge_gap_ && p.z() <= e[k].hi + merge_gap_) {
      const bool grows = p.z() < e[k].lo || p.z() > e[k].hi;
      raise(e[k], p);
      if (grows) {
        mergeNear(column, k);
      }
      return;
    }
  }
  make(column, p);
  if (e.size() > max_levels_) {  // the two closest become one
    std::vector<std::size_t> order(e.size());
    for (std::size_t k = 0; k < e.size(); ++k) {
      order[k] = k;
    }
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return e[a].lo < e[b].lo; });
    std::size_t best = 0;
    float best_gap = std::numeric_limits<float>::infinity();
    for (std::size_t k = 0; k + 1 < order.size(); ++k) {
      const float g = gap(e[order[k]].lo, e[order[k]].hi, e[order[k + 1]].lo, e[order[k + 1]].hi);
      if (g < best_gap) {
        best_gap = g;
        best = k;
      }
    }
    Element & a = e[order[best]];
    const Element & b = e[order[best + 1]];
    a.lo = std::min(a.lo, b.lo);
    a.hi = std::max(a.hi, b.hi);
    a.log_odds = std::max(a.log_odds, b.log_odds);
    a.time = std::max(a.time, b.time);
    a.hit_scan = std::max(a.hit_scan, b.hit_scan);
    e.erase(e.begin() + static_cast<std::ptrdiff_t>(order[best + 1]));
  }
}

bool MultiLevelSurface::crosses(const Element & level, float z_min, float z_max) const
{
  return level.hi + thickness_ >= z_min && level.lo - thickness_ <= z_max;
}

void MultiLevelSurface::draw(const Element & level, std::vector<float> & heights) const
{
  for (float z = level.lo; z < level.hi; z += element_height_) {
    heights.push_back(z);
  }
  heights.push_back(level.hi);
}

}  // namespace sac_perception

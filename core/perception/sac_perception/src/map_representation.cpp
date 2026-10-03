#include "sac_perception/map_representation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sac_perception
{

FreeSpaceParams readFreeSpace(const Params & params)
{
  FreeSpaceParams f;
  f.burn_in = params.getDouble("free_space.burn_in", f.burn_in);
  f.sparsity = params.getDouble("free_space.sparsity", f.sparsity);
  f.static_after = params.getDouble("free_space.static_after", f.static_after);
  return f;
}

void DirectProjection::initialize(const Params & params, const GridGeometry & geometry, const VehicleBox &)
{
  grid_ = std::make_unique<RollingGrid>(geometry.size, geometry.resolution);
  grid_->setRecenterDistance(geometry.recenter_distance);
  grid_->setFreeSpace(readFreeSpace(params));
}

void DirectProjection::insert(int source, const Scan & scan)
{
  if (static_cast<int>(last_.size()) <= source) {
    last_.resize(source + 1);
  }
  auto & last = last_[source];
  last.clear();
  for (const Ray & ray : scan.rays) {
    if (ray.hit && ray.mark) {
      const float top = std::isfinite(ray.ground_z) ? ray.end.z() - ray.ground_z : std::numeric_limits<float>::quiet_NaN();
      last.push_back({ray.end.head<2>(), top, scan.time, grid_->dynamicAt(ray.end.x(), ray.end.y())});
    }
  }
  grid_->integrate(source, scan);  // after: what it takes now is judged by the free space before
  latest_ = std::max(latest_, scan.time);
}

void DirectProjection::recent(double since, std::vector<RecentPoint> & out) const
{
  out.clear();
  for (const auto & last : last_) {
    for (const RecentPoint & p : last) {
      if (p.time >= since) {
        out.push_back(p);
      }
    }
  }
}

void DirectProjection::points(std::vector<MapPoint> & out) const
{
  out.clear();
  const std::vector<int8_t> data = grid_->combined();
  const int w = grid_->width();
  const double res = grid_->resolution();
  for (int j = 0; j < w; ++j) {
    for (int i = 0; i < w; ++i) {
      const int8_t v = data[grid_->index(i, j)];
      if (v <= 50) {
        continue;
      }
      const double x = grid_->originX() + (i + 0.5) * res;
      const double y = grid_->originY() + (j + 0.5) * res;
      float z = 0.0f;
      if (!grid_->groundNear(x, y, 1.0, 0.0, std::numeric_limits<double>::infinity(), z)) {
        continue;  // nowhere to draw it
      }
      out.push_back({Eigen::Vector3f(x, y, z), static_cast<float>(v), 1, 0});
    }
  }
}

}  // namespace sac_perception

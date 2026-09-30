#include "sac_perception/map_representation.hpp"

namespace sac_perception
{

void DirectProjection::initialize(const Params &, const GridGeometry & geometry, const VehicleBox &)
{
  grid_ = std::make_unique<RollingGrid>(geometry.size, geometry.resolution);
  grid_->setRecenterDistance(geometry.recenter_distance);
}

}  // namespace sac_perception

#include "sac_perception/sparse_voxel.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sac_perception
{

void SparseVoxel::configure(const Params & params)
{
  element_height_ = static_cast<float>(params.getDouble("voxel_height", element_height_));
  if (element_height_ <= 0.0f) {
    throw std::invalid_argument("map voxel_height must be positive");
  }
}

void SparseVoxel::add(Column & column, const Eigen::Vector3f & p)
{
  const int iz = std::clamp(static_cast<int>(std::floor(p.z() / element_height_)), -32768, 32767);
  for (Element & voxel : column.elements) {
    if (voxel.iz == iz) {
      raise(voxel, p);
      return;
    }
  }
  make(column, p).iz = static_cast<int16_t>(iz);
}

bool SparseVoxel::crosses(const Element & voxel, float z_min, float z_max) const
{
  return voxel.iz >= static_cast<int>(std::floor(z_min / element_height_)) &&
         voxel.iz <= static_cast<int>(std::floor(z_max / element_height_));
}

void SparseVoxel::draw(const Element & voxel, std::vector<float> & heights) const
{
  heights.push_back((voxel.iz + 0.5f) * element_height_);  // its centre
}

}  // namespace sac_perception

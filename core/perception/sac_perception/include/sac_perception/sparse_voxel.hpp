// sparse_voxel: the obstacle points in voxels of the grid's cell size and `voxel_height`
// [m] (0.2) (column_map.hpp for what the 3D representations share: the modes, the rules,
// the parameters). A point joins the voxel it falls in; a ray lowers the voxels whose height
// it runs through. Each voxel also keeps the lowest and highest point in it, for the band.

#pragma once

#include "sac_perception/column_map.hpp"

namespace sac_perception
{

class SparseVoxel : public ColumnMap
{
protected:
  void configure(const Params & params) override;
  void add(Column & column, const Eigen::Vector3f & p) override;
  bool crosses(const Element & element, float z_min, float z_max) const override;
  void draw(const Element & element, std::vector<float> & heights) const override;
  const char * elementName() const override { return "voxels"; }
};

}  // namespace sac_perception

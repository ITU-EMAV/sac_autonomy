// The car's own box in base_footprint: what the sensors must not take for an obstacle, and
// how high something must be for the car to pass under it. The node reads it from the URDF
// (robot_geometry.hpp) or the YAML and gives it to the filters and the map. No ROS here.

#pragma once

#include <Eigen/Core>

namespace sac_perception
{

struct VehicleBox
{
  Eigen::Vector3f min{0.0f, 0.0f, 0.0f};
  Eigen::Vector3f max{0.0f, 0.0f, 0.0f};
  bool known = false;

  float height() const { return max.z(); }  // over base_footprint (the ground under the car)
};

}  // namespace sac_perception

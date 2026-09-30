// The car's box from its URDF (robot_state_publisher's robot_description): every visual and
// collision shape of every link, placed with the joints at zero (wheels straight, suspension
// at rest), in `base_frame`. Boxes, cylinders and spheres are exact; meshes (STL, OBJ) are
// read for their bounds, a package:// path through the ament index. A shape it cannot read
// (another mesh format, a missing file) counts as its link's origin, and is reported.

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "sac_perception/vehicle.hpp"

namespace sac_perception
{

struct RobotGeometry
{
  VehicleBox box;
  std::vector<std::string> warnings;  // shapes counted by their origin only
};

/// nullopt if the URDF cannot be parsed or has no `base_frame`
std::optional<RobotGeometry> vehicleFromUrdf(const std::string & urdf, const std::string & base_frame);

}  // namespace sac_perception

// Where the car cannot go, from what a 3D representation knows about a column: its ground
// and what is above it. The same rules for every 3D representation (sparse_voxel,
// multi_level_surface), so they differ only in how they keep the data. No ROS here.
//
//   ground                     the column's ground: the lowest level seen there (a bridge's
//                              deck does not replace the road under it), else a
//                              neighbour's within ground_search_radius, else the ground
//                              the ground filter estimated under the obstacle points
//   band                       an obstacle between ground + min_obstacle_height and the top:
//                                max_from: vehicle   the car's height + clearance (default)
//                                          fixed     `max` over the ground
//                              so a deck, a sign or branches over the car are not obstacles,
//                              and a ceiling lower than the car is: a tunnel or a bridge too
//                              low is closed
//   max_step [m] (off)         the ground rising or falling more than this to a neighbouring
//                              cell (a kerb, a ditch): not passable
// Without any known ground an occupied element is an obstacle, as in direct_projection.

#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "sac_perception/params.hpp"
#include "sac_perception/vehicle.hpp"

namespace sac_perception
{

struct Traversability
{
  float min_obstacle_height = 0.25f;  // [m] over the ground: lower is drivable (kerbs, bumps)
  float top = 2.5f;                   // [m] over the ground: higher is overhead
  float max_step = -1.0f;             // [m] < 0: off
  float ground_search_radius = 1.0f;  // [m]
  float ground_max_age = 3.0f;        // [s]
  float level_gap = 1.0f;             // [m] a ground this far over the held one is another level

  void read(const Params & params, const VehicleBox & vehicle)
  {
    min_obstacle_height = static_cast<float>(params.getDouble("min_obstacle_height", min_obstacle_height));
    const float clearance = static_cast<float>(params.getDouble("clearance", 0.3));
    const std::string from = params.getString("max_from", vehicle.known ? "vehicle" : "fixed");
    if (from == "vehicle") {
      if (!vehicle.known) {
        throw std::invalid_argument("map max_from: vehicle needs the car's box (vehicle.from in the node)");
      }
      top = vehicle.height() + clearance;
    } else if (from == "fixed") {
      top = static_cast<float>(params.getDouble("max", top));
    } else {
      throw std::invalid_argument("map max_from: vehicle or fixed, not '" + from + "'");
    }
    max_step = static_cast<float>(params.getDouble("max_step", max_step));
    ground_search_radius = static_cast<float>(params.getDouble("ground_search_radius", ground_search_radius));
    ground_max_age = static_cast<float>(params.getDouble("ground_max_age", ground_max_age));
    level_gap = static_cast<float>(params.getDouble("level_gap", level_gap));
  }

  /// Does an element from z_lo to z_hi block the car, over a column's ground (NaN: unknown)?
  bool blocks(float ground, float z_lo, float z_hi) const
  {
    if (std::isnan(ground)) {
      return true;
    }
    return z_hi > ground + min_obstacle_height && z_lo < ground + top;
  }
};

}  // namespace sac_perception

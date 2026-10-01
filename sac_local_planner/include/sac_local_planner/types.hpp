// What the local planner's pieces share: candidates, the planning context, the obstacles'
// distance map, and the plugin interfaces (a generator makes candidates, cost functions
// score them). No ROS in here.

#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "sac_local_planner/reference_path.hpp"
#include "sac_perception/params.hpp"

namespace sac_local_planner
{

using sac_perception::MapParams;
using sac_perception::Params;

/// A candidate path in the map frame, points every `step` metres
struct Candidate
{
  std::vector<Pose2> points;
  std::vector<double> d;           // offset from the route at each point
  double target_d = 0.0;           // where it settles
  double shift_length = 0.0;       // [m] over which it gets there
  double step = 0.5;               // [m] between points
  // Filled by the collision check
  std::vector<double> clearance;   // [m] footprint to the nearest obstacle at each point
  double min_clearance = std::numeric_limits<double>::infinity();
  double blocked_at = std::numeric_limits<double>::infinity();  // [m] along it, first collision
  bool blocked_by_moving = false;  // the first collision is with a moving object
  // Filled by the costs
  double cost = 0.0;
  std::vector<std::pair<std::string, double>> costs;

  bool free() const { return !std::isfinite(blocked_at); }
};

/// Distance to the nearest obstacle, from the perception grid (in its own frame, odom),
/// with the transform from the map frame into it
struct DistanceMap
{
  std::vector<float> distance;     // [m] per cell, row-major from the origin corner
  int width = 0;
  double resolution = 0.2;
  double origin_x = 0.0;           // grid frame
  double origin_y = 0.0;
  // grid frame <- map: rotation by yaw, then translation
  double yaw = 0.0;
  double tx = 0.0;
  double ty = 0.0;

  bool empty() const { return distance.empty(); }
  /// Distance at a map point; infinity outside the grid (unknown there)
  double at(double x, double y) const
  {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double gx = c * x - s * y + tx;
    const double gy = s * x + c * y + ty;
    const int i = static_cast<int>(std::floor((gx - origin_x) / resolution));
    const int j = static_cast<int>(std::floor((gy - origin_y) / resolution));
    if (i < 0 || j < 0 || i >= width || j >= width) {
      return std::numeric_limits<double>::infinity();
    }
    return distance[static_cast<std::size_t>(j) * width + i];
  }
};

/// Something moving (a person crossing), from the perception's objects, in the map frame
struct MovingObject
{
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();  // [m/s]
  double radius = 0.5;   // [m] its box's half diagonal
  double sigma = 0.2;    // [m] its position's uncertainty (1 sigma)
  bool moving = true;    // the perception calls it moving; else only its speed is known, enough
                         // to yield to a small one crossing, not to drive around it
};

/// Everything a generator or a cost function may use
struct PlanningContext
{
  const ReferencePath * route = nullptr;
  Pose2 ego;                       // base_footprint in map
  double speed = 0.0;              // [m/s]
  double s = 0.0;                  // ego on the route
  double d = 0.0;
  double heading_error = 0.0;      // ego yaw - route yaw at s
  const DistanceMap * obstacles = nullptr;
  const std::vector<MovingObject> * moving = nullptr;  // none: only the grid
  double previous_target_d = 0.0;  // of the last chosen candidate
  bool has_previous = false;
};

class TrajectoryGenerator
{
public:
  virtual ~TrajectoryGenerator() = default;
  virtual void initialize(const Params & params) = 0;
  virtual std::vector<Candidate> generate(const PlanningContext & context) const = 0;
};

class CostFunction
{
public:
  virtual ~CostFunction() = default;
  /// `weight` from the config multiplies what cost() returns
  virtual void initialize(const Params & params) { (void)params; }
  virtual double cost(const Candidate & candidate, const PlanningContext & context) const = 0;
};

}  // namespace sac_local_planner

// The local planner's core, without ROS: candidates from the generator, each checked
// against the obstacles' distance map with the car's footprint, scored by the cost
// functions; the cheapest free one wins. With none free, the one blocked furthest away is
// driven with a speed profile that stops stop_margin before the obstacle.
//
// Footprint: circles along the car (offsets [m] from base_footprint along its heading) of
// one radius; a point collides where a circle comes closer than safety_margin to an
// occupied cell.
// Speed: the route's limit from curvature (max_lateral_acceleration), braking towards lower
// limits (max_deceleration) and a stop, and accelerating from the car's speed
// (max_acceleration).

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "sac_local_planner/types.hpp"

namespace sac_local_planner
{

struct Footprint
{
  std::vector<double> offsets{-0.9, 0.0, 0.9};  // [m] along the car from base_footprint
  double radius = 0.95;                          // [m] covers the 1.66 m wide car
  double safety_margin = 0.3;                    // [m]
};

struct SpeedLimits
{
  double max_speed = 10.0;
  double max_lateral_acceleration = 3.0;
  double max_acceleration = 2.0;
  double max_deceleration = 4.0;
  double stop_margin = 3.0;  // [m] stop this far before an obstacle
};

struct WeightedCost
{
  std::string name;
  double weight = 1.0;
  std::shared_ptr<CostFunction> function;
};

struct PlanResult
{
  std::vector<Candidate> candidates;
  int chosen = -1;
  std::vector<double> speeds;  // at the chosen candidate's points
  bool stopping = false;
};

class LocalPlanner
{
public:
  LocalPlanner(
    std::shared_ptr<TrajectoryGenerator> generator, std::vector<WeightedCost> costs, Footprint footprint,
    SpeedLimits limits);

  PlanResult plan(const PlanningContext & context) const;

  /// Fills clearance, min_clearance and blocked_at
  void check(Candidate & candidate, const DistanceMap & obstacles) const;
  /// Speeds along a candidate from the car's speed, stopping at stop_at [m] along it (or not)
  std::vector<double> speedProfile(const Candidate & candidate, double speed, double stop_at) const;

  const SpeedLimits & limits() const { return limits_; }

private:
  std::shared_ptr<TrajectoryGenerator> generator_;
  std::vector<WeightedCost> costs_;
  Footprint footprint_;
  SpeedLimits limits_;
};

}  // namespace sac_local_planner

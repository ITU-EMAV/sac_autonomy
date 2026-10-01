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
//
// Moving objects (a person crossing): the grid shows where they are, not where they will be.
// Each candidate is driven in time with its speed profile (as if free), and each moving
// object goes on at its velocity: a point of the candidate collides where, around the time
// the car would be there (+- time_margin), the object comes within the footprint's radius,
// its own, `safety` and its uncertainty (its position's, growing with time at
// speed_uncertainty, up to max_uncertainty), within `horizon` seconds. One walking along the
// road leaves a side free: the car passes them.
// Yielding: one crossing the road (moving across the route faster than `crossing_speed`) that
// is in the corridor (`corridor` [m] each side of the route) or enters it before the car could
// be there (+ time_margin) blocks every candidate where it crosses: the car does not swerve in
// front of them, it stops stop_margin before, and drives on once they have left the
// corridor. For yielding, a small object (up to `small_size` [m]) with a sure speed (over
// 2 sigma of it) counts though the perception does not call it moving yet: it does so once
// the object takes cells seen free a moment before, which the lidar's rings may not show soon
// enough on the road far ahead; a barrier sliced by a ring slides along the road, never
// across it.

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

struct MovingLimits
{
  bool enabled = true;
  double time_margin = 1.0;        // [s] around the car's arrival
  double horizon = 6.0;            // [s] how far ahead objects are followed
  double safety = 0.5;             // [m]
  double speed_uncertainty = 0.3;  // [m/s] its position's uncertainty grows by this
  double max_uncertainty = 1.5;    // [m]
  double corridor = 4.0;           // [m] each side of the route: where a crossing one is in the way
  double crossing_speed = 0.3;     // [m/s] across the route: crossing
};

/// Which of the perception's objects the planner takes: moving ones, and small ones whose
/// speed is sure (for yielding)
struct ObjectSelection
{
  bool enabled = false;      // off by default: only what the perception calls moving
  double small_size = 1.2;   // [m] longest side
  double min_speed = 0.5;    // [m/s] its speed less 2 sigma
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
    SpeedLimits limits, MovingLimits moving = MovingLimits{});

  PlanResult plan(const PlanningContext & context) const;

  /// Fills clearance, min_clearance and blocked_at
  void check(Candidate & candidate, const DistanceMap & obstacles) const;
  /// The moving objects against a candidate driven in time from the car's speed: lowers
  /// blocked_at where one would meet it (and sets blocked_by_moving)
  void checkMoving(Candidate & candidate, const std::vector<MovingObject> & objects, double speed) const;
  /// Where along the route [m from the car] the first person crossing it is to be yielded to
  /// (infinity: none)
  double yieldAt(const PlanningContext & context) const;
  /// Speeds along a candidate from the car's speed, stopping at stop_at [m] along it (or not)
  std::vector<double> speedProfile(const Candidate & candidate, double speed, double stop_at) const;

  const SpeedLimits & limits() const { return limits_; }

private:
  std::shared_ptr<TrajectoryGenerator> generator_;
  std::vector<WeightedCost> costs_;
  Footprint footprint_;
  SpeedLimits limits_;
  MovingLimits moving_;
};

}  // namespace sac_local_planner

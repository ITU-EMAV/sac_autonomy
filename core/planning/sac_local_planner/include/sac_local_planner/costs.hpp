// The built-in cost functions (plugins); the planner adds them up, each times its `weight`.
//   obstacle_clearance: (desired_clearance - the candidate's smallest clearance)^2 where it
//     comes closer than desired_clearance [m] (1.0): keeps a margin, prefers the wider gap
//   offset_from_route: mean d^2 over the candidate: back to the route when it can
//   lateral_acceleration: (speed^2 x the candidate's largest curvature)^2, speed at least
//     min_speed [m/s] (3): smooth, gentle swerves
//   consistency: (target offset - last cycle's)^2: no hopping between gaps

#pragma once

#include "sac_local_planner/types.hpp"

namespace sac_local_planner
{

class ObstacleClearance : public CostFunction
{
public:
  void initialize(const Params & params) override;
  double cost(const Candidate & candidate, const PlanningContext & context) const override;

private:
  double desired_clearance_ = 1.0;
};

class OffsetFromRoute : public CostFunction
{
public:
  double cost(const Candidate & candidate, const PlanningContext & context) const override;
};

class LateralAcceleration : public CostFunction
{
public:
  void initialize(const Params & params) override;
  double cost(const Candidate & candidate, const PlanningContext & context) const override;

private:
  double min_speed_ = 3.0;
};

class Consistency : public CostFunction
{
public:
  double cost(const Candidate & candidate, const PlanningContext & context) const override;
};

}  // namespace sac_local_planner

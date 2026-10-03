#include "sac_local_planner/costs.hpp"

#include <algorithm>
#include <cmath>

namespace sac_local_planner
{

void ObstacleClearance::initialize(const Params & params)
{
  desired_clearance_ = params.getDouble("desired_clearance", desired_clearance_);
}

double ObstacleClearance::cost(const Candidate & candidate, const PlanningContext &) const
{
  const double short_of = desired_clearance_ - candidate.min_clearance;
  return short_of > 0.0 ? short_of * short_of : 0.0;
}

double OffsetFromRoute::cost(const Candidate & candidate, const PlanningContext &) const
{
  if (candidate.d.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (double d : candidate.d) {
    sum += d * d;
  }
  return sum / static_cast<double>(candidate.d.size());
}

void LateralAcceleration::initialize(const Params & params)
{
  min_speed_ = params.getDouble("min_speed", min_speed_);
}

double LateralAcceleration::cost(const Candidate & candidate, const PlanningContext & context) const
{
  double curvature = 0.0;
  for (const Pose2 & p : candidate.points) {
    curvature = std::max(curvature, std::abs(p.curvature));
  }
  const double v = std::max(context.speed, min_speed_);
  const double acceleration = v * v * curvature;
  return acceleration * acceleration;
}

double Consistency::cost(const Candidate & candidate, const PlanningContext & context) const
{
  if (!context.has_previous) {
    return 0.0;
  }
  const double change = candidate.target_d - context.previous_target_d;
  return change * change;
}

}  // namespace sac_local_planner

#include "sac_local_planner/local_planner.hpp"

#include <algorithm>
#include <cmath>

namespace sac_local_planner
{

LocalPlanner::LocalPlanner(
  std::shared_ptr<TrajectoryGenerator> generator, std::vector<WeightedCost> costs, Footprint footprint,
  SpeedLimits limits)
: generator_(std::move(generator)), costs_(std::move(costs)), footprint_(std::move(footprint)), limits_(limits)
{
}

void LocalPlanner::check(Candidate & candidate, const DistanceMap & obstacles) const
{
  const std::size_t n = candidate.points.size();
  candidate.clearance.assign(n, std::numeric_limits<double>::infinity());
  candidate.min_clearance = std::numeric_limits<double>::infinity();
  candidate.blocked_at = std::numeric_limits<double>::infinity();
  if (obstacles.empty()) {
    return;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const Pose2 & p = candidate.points[i];
    const double c = std::cos(p.yaw);
    const double s = std::sin(p.yaw);
    double clearance = std::numeric_limits<double>::infinity();
    for (double offset : footprint_.offsets) {
      clearance = std::min(clearance, obstacles.at(p.x + offset * c, p.y + offset * s) - footprint_.radius);
    }
    candidate.clearance[i] = clearance;
    candidate.min_clearance = std::min(candidate.min_clearance, clearance);
    if (clearance < footprint_.safety_margin && !std::isfinite(candidate.blocked_at)) {
      candidate.blocked_at = static_cast<double>(i) * candidate.step;
    }
  }
}

std::vector<double> LocalPlanner::speedProfile(const Candidate & candidate, double speed, double stop_at) const
{
  const std::size_t n = candidate.points.size();
  std::vector<double> v(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double curvature = std::max(std::abs(candidate.points[i].curvature), 1e-6);
    v[i] = std::min(limits_.max_speed, std::sqrt(limits_.max_lateral_acceleration / curvature));
    if (static_cast<double>(i) * candidate.step >= stop_at) {
      v[i] = 0.0;
    }
  }
  // Braking towards what comes: backwards
  for (std::size_t i = n; i-- > 1;) {
    v[i - 1] = std::min(v[i - 1], std::sqrt(v[i] * v[i] + 2.0 * limits_.max_deceleration * candidate.step));
  }
  // Accelerating from the car's speed: forwards
  double previous = std::max(0.0, speed);
  for (std::size_t i = 0; i < n; ++i) {
    v[i] = std::min(v[i], std::sqrt(previous * previous + 2.0 * limits_.max_acceleration * candidate.step));
    previous = v[i];
  }
  return v;
}

PlanResult LocalPlanner::plan(const PlanningContext & context) const
{
  PlanResult result;
  result.candidates = generator_->generate(context);
  if (result.candidates.empty()) {
    return result;
  }
  for (Candidate & c : result.candidates) {
    if (context.obstacles != nullptr) {
      check(c, *context.obstacles);
    }
    c.cost = 0.0;
    c.costs.clear();
    for (const WeightedCost & w : costs_) {
      const double value = w.weight * w.function->cost(c, context);
      c.costs.emplace_back(w.name, value);
      c.cost += value;
    }
  }
  // The cheapest free candidate; with none, the one blocked furthest away, then cheapest
  int best = -1;
  for (int i = 0; i < static_cast<int>(result.candidates.size()); ++i) {
    const Candidate & c = result.candidates[i];
    if (best < 0) {
      best = i;
      continue;
    }
    const Candidate & b = result.candidates[best];
    if (c.free() != b.free()) {
      if (c.free()) {
        best = i;
      }
    } else if (!c.free() && std::abs(c.blocked_at - b.blocked_at) > 1e-9) {
      if (c.blocked_at > b.blocked_at) {
        best = i;
      }
    } else if (c.cost < b.cost) {
      best = i;
    }
  }
  result.chosen = best;
  const Candidate & chosen = result.candidates[best];
  result.stopping = !chosen.free();
  const double stop_at = result.stopping ? std::max(0.0, chosen.blocked_at - limits_.stop_margin) :
    std::numeric_limits<double>::infinity();
  result.speeds = speedProfile(chosen, context.speed, stop_at);
  return result;
}

}  // namespace sac_local_planner

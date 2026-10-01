#include "sac_local_planner/local_planner.hpp"

#include <algorithm>
#include <cmath>

namespace sac_local_planner
{

LocalPlanner::LocalPlanner(
  std::shared_ptr<TrajectoryGenerator> generator, std::vector<WeightedCost> costs, Footprint footprint,
  SpeedLimits limits, MovingLimits moving)
: generator_(std::move(generator)), costs_(std::move(costs)), footprint_(std::move(footprint)), limits_(limits),
  moving_(moving)
{
}

namespace
{
/// Distance from p to the segment a-b
double toSegment(const Eigen::Vector2d & p, const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  const Eigen::Vector2d ab = b - a;
  const double length2 = ab.squaredNorm();
  const double t = length2 > 1e-12 ? std::clamp((p - a).dot(ab) / length2, 0.0, 1.0) : 0.0;
  return (p - (a + t * ab)).norm();
}
}  // namespace

void LocalPlanner::checkMoving(Candidate & candidate, const std::vector<MovingObject> & objects, double speed) const
{
  if (objects.empty() || candidate.points.empty()) {
    return;
  }
  // When the car would be at each point, driving it free
  const std::vector<double> v = speedProfile(candidate, speed, std::numeric_limits<double>::infinity());
  double t = 0.0;
  for (std::size_t i = 0; i < candidate.points.size(); ++i) {
    if (i > 0) {
      t += candidate.step / std::max(0.1, 0.5 * (v[i - 1] + v[i]));
    }
    const double along = static_cast<double>(i) * candidate.step;
    if (t - moving_.time_margin > moving_.horizon || along >= candidate.blocked_at) {
      return;  // beyond what is followed, or blocked by the grid before
    }
    const Pose2 & p = candidate.points[i];
    const Eigen::Vector2d heading(std::cos(p.yaw), std::sin(p.yaw));
    const double from = std::max(0.0, t - moving_.time_margin);
    const double to = std::min(moving_.horizon, t + moving_.time_margin);
    for (const MovingObject & o : objects) {
      if (!o.moving) {
        continue;  // only its speed is sure: for yielding
      }
      const Eigen::Vector2d a = o.position + o.velocity * from;
      const Eigen::Vector2d b = o.position + o.velocity * to;
      const double reach = footprint_.radius + o.radius + moving_.safety + o.sigma +
        std::min(moving_.max_uncertainty, moving_.speed_uncertainty * to);
      for (double offset : footprint_.offsets) {
        if (toSegment(Eigen::Vector2d(p.x, p.y) + offset * heading, a, b) < reach) {
          candidate.blocked_at = along;
          candidate.blocked_by_moving = true;
          return;
        }
      }
    }
  }
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

double LocalPlanner::yieldAt(const PlanningContext & context) const
{
  double nearest = std::numeric_limits<double>::infinity();
  if (!moving_.enabled || context.moving == nullptr || context.route == nullptr) {
    return nearest;
  }
  const ReferencePath & route = *context.route;
  for (const MovingObject & o : *context.moving) {
    const Eigen::Vector2d sd = route.toFrenet(o.position, context.s, 80.0);
    double ahead = sd.x() - context.s;
    if (route.closed()) {
      ahead = route.wrap(ahead + route.length() / 2.0) - route.length() / 2.0;
    }
    if (ahead < -2.0) {
      continue;  // behind the car
    }
    const double yaw = route.at(sd.x()).yaw;
    const double across = -std::sin(yaw) * o.velocity.x() + std::cos(yaw) * o.velocity.y();
    if (std::abs(across) < moving_.crossing_speed) {
      continue;  // along the road: the candidates go around it
    }
    // When it is within the corridor: [enter, leave] (in it already: enter <= 0)
    const double edge = moving_.corridor + o.radius;
    double enter = (std::copysign(edge, -across) - sd.y()) / across;
    double leave = (std::copysign(edge, across) - sd.y()) / across;
    if (leave < 0.0) {
      continue;  // it has crossed
    }
    // The soonest the car could be there: speeding up to max_speed
    const double v = std::max(0.0, context.speed);
    const double a = limits_.max_acceleration;
    const double to_max = (limits_.max_speed - v) / a;
    const double d_max = v * to_max + 0.5 * a * to_max * to_max;
    const double distance = std::max(0.0, ahead);
    const double arrive = distance <= d_max ? (-v + std::sqrt(v * v + 2.0 * a * distance)) / a :
      to_max + (distance - d_max) / limits_.max_speed;
    if (enter > arrive + moving_.time_margin || enter > moving_.horizon) {
      continue;  // the car is past before it gets there
    }
    // Stop before the car's front and the person meet
    const double front = footprint_.offsets.empty() ? 0.0 :
      *std::max_element(footprint_.offsets.begin(), footprint_.offsets.end());
    nearest = std::min(nearest, std::max(0.0, ahead - front - footprint_.radius - o.radius - moving_.safety));
  }
  return nearest;
}

PlanResult LocalPlanner::plan(const PlanningContext & context) const
{
  PlanResult result;
  result.candidates = generator_->generate(context);
  if (result.candidates.empty()) {
    return result;
  }
  const double yield = yieldAt(context);
  for (Candidate & c : result.candidates) {
    if (context.obstacles != nullptr) {
      check(c, *context.obstacles);
    }
    if (moving_.enabled && context.moving != nullptr) {
      checkMoving(c, *context.moving, context.speed);
    }
    if (yield < c.blocked_at) {  // someone crossing: no candidate goes on past them
      c.blocked_at = yield;
      c.blocked_by_moving = true;
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

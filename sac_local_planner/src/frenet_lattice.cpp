#include "sac_local_planner/frenet_lattice.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

namespace sac_local_planner
{

namespace
{
double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

Quintic::Quintic(double d0, double slope0, double d1, double length)
: a0_(d0), a1_(slope0)
{
  const double L = std::max(length, 1e-3);
  Eigen::Matrix3d A;
  A << L * L * L, L * L * L * L, L * L * L * L * L,
    3 * L * L, 4 * L * L * L, 5 * L * L * L * L,
    6 * L, 12 * L * L, 20 * L * L * L;
  const Eigen::Vector3d b(d1 - d0 - slope0 * L, -slope0, 0.0);
  const Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  a3_ = x(0);
  a4_ = x(1);
  a5_ = x(2);
}

double Quintic::value(double s) const
{
  return a0_ + a1_ * s + s * s * s * (a3_ + s * (a4_ + s * a5_));
}

double Quintic::slope(double s) const
{
  return a1_ + s * s * (3 * a3_ + s * (4 * a4_ + 5 * a5_ * s));
}

void FrenetLattice::initialize(const Params & params)
{
  max_offset_ = params.getDouble("max_offset", max_offset_);
  offset_step_ = params.getDouble("offset_step", offset_step_);
  shift_times_ = params.getDoubles("shift_times", shift_times_);
  min_shift_length_ = params.getDouble("min_shift_length", min_shift_length_);
  horizon_time_ = params.getDouble("horizon_time", horizon_time_);
  min_horizon_ = params.getDouble("min_horizon", min_horizon_);
  step_ = params.getDouble("step", step_);
  if (offset_step_ <= 0.0 || step_ <= 0.0 || shift_times_.empty()) {
    throw std::invalid_argument("frenet_lattice: offset_step, step > 0 and shift_times");
  }
}

Candidate FrenetLattice::make(
  const PlanningContext & context, double target_d, double shift_length, double horizon) const
{
  const ReferencePath & route = *context.route;
  const double slope0 = std::clamp(std::tan(context.heading_error), -1.0, 1.0);
  const Quintic lateral(context.d, slope0, target_d, shift_length);
  Candidate c;
  c.target_d = target_d;
  c.shift_length = shift_length;
  c.step = step_;
  const int n = static_cast<int>(horizon / step_) + 1;
  std::vector<Eigen::Vector2d> xy;
  xy.reserve(n);
  for (int i = 0; i < n; ++i) {
    const double ds = i * step_;
    if (!route.closed() && context.s + ds > route.length()) {
      break;  // the end of an open route
    }
    const double d = ds < shift_length ? lateral.value(ds) : target_d;
    xy.push_back(route.toCartesian(context.s + ds, d));
    c.d.push_back(d);
  }
  const std::size_t m = xy.size();
  c.points.resize(m);
  for (std::size_t i = 0; i < m; ++i) {
    c.points[i].x = xy[i].x();
    c.points[i].y = xy[i].y();
    const std::size_t a = i > 0 ? i - 1 : 0;
    const std::size_t b = std::min(m - 1, i + 1);
    c.points[i].yaw = b > a ? std::atan2(xy[b].y() - xy[a].y(), xy[b].x() - xy[a].x()) : context.ego.yaw;
  }
  // Curvature from the heading change over two steps on each side
  for (std::size_t i = 0; i < m; ++i) {
    const std::size_t a = i >= 2 ? i - 2 : 0;
    const std::size_t b = std::min(m - 1, i + 2);
    double length = 0.0;
    for (std::size_t k = a; k < b; ++k) {
      length += (xy[k + 1] - xy[k]).norm();
    }
    c.points[i].curvature = length > 1e-6 ? wrapAngle(c.points[b].yaw - c.points[a].yaw) / length : 0.0;
  }
  return c;
}

std::vector<Candidate> FrenetLattice::generate(const PlanningContext & context) const
{
  std::vector<Candidate> candidates;
  if (context.route == nullptr || context.route->empty()) {
    return candidates;
  }
  const double horizon = std::max(min_horizon_, context.speed * horizon_time_);
  const int offsets = static_cast<int>(std::floor(max_offset_ / offset_step_ + 1e-9));
  for (int k = -offsets; k <= offsets; ++k) {
    for (double time : shift_times_) {
      const double shift = std::min(horizon, std::max(min_shift_length_, context.speed * time));
      candidates.push_back(make(context, k * offset_step_, shift, horizon));
    }
  }
  return candidates;
}

}  // namespace sac_local_planner

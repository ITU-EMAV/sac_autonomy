#include "sac_local_planner/reference_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sac_local_planner
{

namespace
{
double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

void ReferencePath::build(const std::vector<Eigen::Vector2d> & points)
{
  points_ = points;
  s_.clear();
  yaw_.clear();
  curvature_.clear();
  length_ = 0.0;
  closed_ = false;
  if (points_.size() < 2) {
    return;
  }
  std::vector<double> steps;
  for (std::size_t i = 1; i < points_.size(); ++i) {
    steps.push_back((points_[i] - points_[i - 1]).norm());
  }
  std::vector<double> sorted = steps;
  std::nth_element(sorted.begin(), sorted.begin() + sorted.size() / 2, sorted.end());
  const double spacing = sorted[sorted.size() / 2];
  closed_ = (points_.back() - points_.front()).norm() < 3.0 * spacing;
  if (closed_) {
    points_.push_back(points_.front());  // the closing segment
    steps.push_back((points_.back() - points_[points_.size() - 2]).norm());
  }
  s_.push_back(0.0);
  for (double step : steps) {
    s_.push_back(s_.back() + step);
  }
  length_ = s_.back();
  const std::size_t n = points_.size();
  for (std::size_t i = 0; i + 1 < n; ++i) {
    const Eigen::Vector2d d = points_[i + 1] - points_[i];
    yaw_.push_back(std::atan2(d.y(), d.x()));
  }
  yaw_.push_back(yaw_.back());
  // Curvature from the heading change over a few metres on each side (smooth)
  const int k = std::max(1, static_cast<int>(std::lround(3.0 / std::max(spacing, 1e-3))));
  curvature_.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    long a = static_cast<long>(i) - k;
    long b = static_cast<long>(i) + k;
    const long last = static_cast<long>(n) - 2;  // the segments' last index
    if (closed_) {
      a = ((a % (last + 1)) + (last + 1)) % (last + 1);
      b = b % (last + 1);
    } else {
      a = std::max(0L, a);
      b = std::min(last, b);
    }
    double ds = s_[b] - s_[a];
    if (closed_ && ds <= 0.0) {
      ds += length_;
    }
    curvature_[i] = ds > 1e-6 ? wrapAngle(yaw_[b] - yaw_[a]) / ds : 0.0;
  }
}

double ReferencePath::wrap(double s) const
{
  if (closed_) {
    s = std::fmod(s, length_);
    return s < 0.0 ? s + length_ : s;
  }
  return std::clamp(s, 0.0, length_);
}

std::size_t ReferencePath::segmentAt(double s) const
{
  const auto it = std::upper_bound(s_.begin(), s_.end(), s);
  const std::size_t i = it == s_.begin() ? 0 : static_cast<std::size_t>(it - s_.begin()) - 1;
  return std::min(i, points_.size() - 2);
}

Pose2 ReferencePath::at(double s) const
{
  s = wrap(s);
  const std::size_t i = segmentAt(s);
  const double t = (s - s_[i]) / std::max(s_[i + 1] - s_[i], 1e-9);
  const Eigen::Vector2d p = points_[i] + t * (points_[i + 1] - points_[i]);
  Pose2 pose;
  pose.x = p.x();
  pose.y = p.y();
  pose.yaw = yaw_[i];
  pose.curvature = (1.0 - t) * curvature_[i] + t * curvature_[i + 1];
  return pose;
}

Eigen::Vector2d ReferencePath::toCartesian(double s, double d) const
{
  const Pose2 p = at(s);
  return {p.x - d * std::sin(p.yaw), p.y + d * std::cos(p.yaw)};
}

Eigen::Vector2d ReferencePath::toFrenet(const Eigen::Vector2d & point, double hint, double window) const
{
  double best = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  double best_d = 0.0;
  auto consider = [&](std::size_t i) {
    const Eigen::Vector2d a = points_[i];
    const Eigen::Vector2d ab = points_[i + 1] - a;
    const double length2 = std::max(ab.squaredNorm(), 1e-12);
    const double t = std::clamp((point - a).dot(ab) / length2, 0.0, 1.0);
    const Eigen::Vector2d closest = a + t * ab;
    const double dist = (point - closest).norm();
    if (dist < best) {
      best = dist;
      best_s = s_[i] + t * std::sqrt(length2);
      const double cross = ab.x() * (point.y() - a.y()) - ab.y() * (point.x() - a.x());
      best_d = cross >= 0.0 ? dist : -dist;
    }
  };
  const std::size_t segments = points_.size() - 1;
  if (hint < 0.0) {
    for (std::size_t i = 0; i < segments; ++i) {
      consider(i);
    }
  } else {
    // Segments from hint - window/4 to hint + window
    const double from = hint - window / 4.0;
    const double to = hint + window;
    for (double s = from; s <= to; ) {
      const std::size_t i = segmentAt(wrap(s));
      consider(i);
      s += std::max(s_[i + 1] - s_[i], 1e-3);
      if (!closed_ && s > length_) {
        break;
      }
    }
  }
  return {best_s, best_d};
}

}  // namespace sac_local_planner

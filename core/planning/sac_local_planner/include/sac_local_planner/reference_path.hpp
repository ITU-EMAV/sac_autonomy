// The route as a Frenet frame: a point is described by how far along the route it is (s)
// and how far to its left (d). Candidate paths are made in (s, d) and turned back into map
// coordinates here. No ROS in here.

#pragma once

#include <vector>

#include <Eigen/Core>

namespace sac_local_planner
{

struct Pose2
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double curvature = 0.0;  // [1/m], positive left
};

class ReferencePath
{
public:
  /// From the route's points (map frame, ~evenly spaced). A loop when its last point is
  /// within 3 spacings of its first (as the controller decides).
  void build(const std::vector<Eigen::Vector2d> & points);
  bool empty() const { return points_.size() < 2; }
  bool closed() const { return closed_; }
  double length() const { return length_; }

  /// s wrapped onto a loop, clamped onto an open route
  double wrap(double s) const;
  /// The route's pose and curvature at s
  Pose2 at(double s) const;
  /// Map point of (s, d)
  Eigen::Vector2d toCartesian(double s, double d) const;
  /// (s, d) of a map point: the nearest point of the route. With a hint (the last s), only
  /// a window around it is searched, so a route that passes close to itself is followed in
  /// order.
  Eigen::Vector2d toFrenet(const Eigen::Vector2d & point, double hint = -1.0, double window = 30.0) const;

private:
  std::size_t segmentAt(double s) const;

  std::vector<Eigen::Vector2d> points_;
  std::vector<double> s_;          // at each point
  std::vector<double> yaw_;        // of the segment from each point
  std::vector<double> curvature_;  // at each point (from its neighbours)
  double length_ = 0.0;
  bool closed_ = false;
};

}  // namespace sac_local_planner

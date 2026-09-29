// The local planner's core: the Frenet frame, the quintic shifts, going around an obstacle
// on the route, and stopping before a road that is closed.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <memory>

#include "sac_local_planner/costs.hpp"
#include "sac_local_planner/frenet_lattice.hpp"
#include "sac_local_planner/local_planner.hpp"
#include "sac_perception/grid.hpp"

using namespace sac_local_planner;

namespace
{
ReferencePath straight(double length = 200.0)
{
  std::vector<Eigen::Vector2d> points;
  for (double x = 0.0; x <= length; x += 0.5) {
    points.emplace_back(x, 0.0);
  }
  ReferencePath r;
  r.build(points);
  return r;
}

ReferencePath circle(double radius = 50.0)
{
  std::vector<Eigen::Vector2d> points;
  const int n = static_cast<int>(2 * M_PI * radius / 0.5);
  for (int i = 0; i < n; ++i) {
    const double a = 2 * M_PI * i / n;
    points.emplace_back(radius * std::cos(a), radius * std::sin(a));
  }
  ReferencePath r;
  r.build(points);
  return r;
}

/// A distance map (map frame = grid frame) with the given rectangles occupied
DistanceMap obstaclesAt(const std::vector<std::array<double, 4>> & boxes)
{
  DistanceMap m;
  m.width = 500;
  m.resolution = 0.2;
  m.origin_x = -20.0;
  m.origin_y = -50.0;
  std::vector<uint8_t> occupied(m.width * m.width, 0);
  for (int j = 0; j < m.width; ++j) {
    for (int i = 0; i < m.width; ++i) {
      const double x = m.origin_x + (i + 0.5) * m.resolution;
      const double y = m.origin_y + (j + 0.5) * m.resolution;
      for (const auto & b : boxes) {
        if (x >= b[0] && x <= b[1] && y >= b[2] && y <= b[3]) {
          occupied[j * m.width + i] = 1;
        }
      }
    }
  }
  m.distance = sac_perception::distanceTransform(occupied, m.width, m.resolution);
  return m;
}

LocalPlanner planner()
{
  MapParams none;
  auto lattice = std::make_shared<FrenetLattice>();
  lattice->initialize(none);
  auto clearance = std::make_shared<ObstacleClearance>();
  clearance->initialize(none);
  auto lateral = std::make_shared<LateralAcceleration>();
  lateral->initialize(none);
  std::vector<WeightedCost> costs = {
    {"obstacle_clearance", 50.0, clearance},
    {"offset_from_route", 1.0, std::make_shared<OffsetFromRoute>()},
    {"lateral_acceleration", 0.1, lateral},
    {"consistency", 2.0, std::make_shared<Consistency>()}};
  return LocalPlanner(lattice, costs, Footprint{}, SpeedLimits{});
}

PlanningContext at(const ReferencePath & route, double s, double d, double speed, const DistanceMap * obstacles)
{
  PlanningContext c;
  c.route = &route;
  const Pose2 p = route.at(s);
  const Eigen::Vector2d xy = route.toCartesian(s, d);
  c.ego = {xy.x(), xy.y(), p.yaw, 0.0};
  c.s = s;
  c.d = d;
  c.speed = speed;
  c.obstacles = obstacles;
  return c;
}
}  // namespace

TEST(ReferencePath, FrenetRoundTrip)
{
  for (const ReferencePath & r : {straight(), circle()}) {
    for (double s : {3.0, 57.3, 150.2}) {
      for (double d : {-2.5, 0.0, 1.7}) {
        const Eigen::Vector2d sd = r.toFrenet(r.toCartesian(s, d));
        EXPECT_NEAR(sd.x(), r.wrap(s), 0.05) << s << " " << d;
        EXPECT_NEAR(sd.y(), d, 0.02) << s << " " << d;
      }
    }
  }
}

TEST(ReferencePath, ALoopWrapsAndKnowsItsCurvature)
{
  const ReferencePath r = circle(50.0);
  EXPECT_TRUE(r.closed());
  EXPECT_NEAR(r.length(), 2 * M_PI * 50.0, 0.5);
  EXPECT_NEAR(r.wrap(r.length() + 3.0), 3.0, 1e-6);
  EXPECT_NEAR(r.at(100.0).curvature, 1.0 / 50.0, 0.002);
  // Across the start: a hint just before the end finds a point just after the start
  const Eigen::Vector2d sd = r.toFrenet(r.toCartesian(2.0, 0.5), r.length() - 3.0);
  EXPECT_NEAR(sd.x(), 2.0, 0.05);
}

TEST(FrenetLattice, QuinticMeetsItsEnds)
{
  const Quintic q(1.0, 0.2, -2.0, 15.0);
  EXPECT_NEAR(q.value(0.0), 1.0, 1e-9);
  EXPECT_NEAR(q.slope(0.0), 0.2, 1e-9);
  EXPECT_NEAR(q.value(15.0), -2.0, 1e-6);
  EXPECT_NEAR(q.slope(15.0), 0.0, 1e-6);
}

TEST(LocalPlanner, StaysOnTheRouteWithNothingOnIt)
{
  const ReferencePath route = straight();
  const DistanceMap empty = obstaclesAt({});
  const PlanResult r = planner().plan(at(route, 10.0, 0.0, 8.0, &empty));
  ASSERT_GE(r.chosen, 0);
  EXPECT_FALSE(r.stopping);
  EXPECT_NEAR(r.candidates[r.chosen].target_d, 0.0, 1e-9);
}

TEST(LocalPlanner, GoesAroundABoxOnTheRoute)
{
  // A 1 x 1 m box on the route 20 m ahead
  const ReferencePath route = straight();
  const DistanceMap box = obstaclesAt({{29.5, 30.5, -0.5, 0.5}});
  const PlanResult r = planner().plan(at(route, 10.0, 0.0, 8.0, &box));
  ASSERT_GE(r.chosen, 0);
  const Candidate & c = r.candidates[r.chosen];
  EXPECT_FALSE(r.stopping);
  EXPECT_TRUE(c.free());
  EXPECT_GT(c.min_clearance, 0.3);                      // the safety margin
  EXPECT_GT(std::abs(c.target_d), 1.5);                 // beside it
  EXPECT_GT(*std::min_element(r.speeds.begin(), r.speeds.end()), 1.0);  // no stop
}

TEST(LocalPlanner, StopsBeforeAClosedRoad)
{
  // A wall across the whole corridor 25 m ahead
  const ReferencePath route = straight();
  const DistanceMap wall = obstaclesAt({{35.0, 35.5, -10.0, 10.0}});
  const PlanResult r = planner().plan(at(route, 10.0, 0.0, 8.0, &wall));
  ASSERT_GE(r.chosen, 0);
  EXPECT_TRUE(r.stopping);
  const Candidate & c = r.candidates[r.chosen];
  // Speed 0 from stop_margin before where it would touch the wall
  for (std::size_t i = 0; i < c.points.size(); ++i) {
    if (i * c.step >= c.blocked_at - 3.0) {
      EXPECT_DOUBLE_EQ(r.speeds[i], 0.0);
    }
  }
  // Where it comes to rest: stop_margin + footprint before the wall
  EXPECT_LT(c.blocked_at, 25.0);
  EXPECT_GT(c.blocked_at, 20.0);
}

TEST(LocalPlanner, SpeedProfileBrakesAndAccelerates)
{
  const ReferencePath route = straight();
  const LocalPlanner p = planner();
  const PlanningContext context = at(route, 10.0, 0.0, 2.0, nullptr);
  const Candidate c = FrenetLattice().make(context, 0.0, 10.0, 40.0);
  const std::vector<double> v = p.speedProfile(c, 2.0, 30.0);
  EXPECT_LE(v.front(), std::sqrt(2.0 * 2.0 + 2 * 2.0 * 0.5) + 1e-9);  // from the car's speed
  EXPECT_DOUBLE_EQ(v[static_cast<std::size_t>(30.0 / c.step)], 0.0);
  for (std::size_t i = 1; i < v.size(); ++i) {
    // Never harder than the limits
    EXPECT_LE(v[i] * v[i] - v[i - 1] * v[i - 1], 2 * 2.0 * c.step + 1e-9);
    EXPECT_GE(v[i] * v[i] - v[i - 1] * v[i - 1], -2 * 4.0 * c.step - 1e-9);
  }
}

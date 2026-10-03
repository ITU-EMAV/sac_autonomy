// Objects: clusters of the cells seen lately, and tracks following them with a velocity.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "sac_perception/objects.hpp"

using namespace sac_perception;

namespace
{
constexpr bool kNever = false;  // takes no cell known free

/// The cells of a box's outline as a lidar sees it (its sides, every 0.2 m), with noise;
/// `dynamic`: its cells were known free (a moving thing's)
void box(
  std::vector<RecentPoint> & out, Eigen::Vector2f centre, float length, float width, float top, double time,
  std::mt19937 * rng = nullptr, bool dynamic = kNever)
{
  std::normal_distribution<float> noise(0.0f, 0.03f);
  auto add = [&](float x, float y) {
    Eigen::Vector2f p = centre + Eigen::Vector2f(x, y);
    if (rng) {
      p += Eigen::Vector2f(noise(*rng), noise(*rng));
    }
    out.push_back({p, top, time, dynamic});
  };
  for (float x = -length / 2; x <= length / 2 + 1e-3f; x += 0.2f) {
    add(x, -width / 2);
    add(x, width / 2);
  }
  for (float y = -width / 2 + 0.2f; y < width / 2 - 1e-3f; y += 0.2f) {
    add(-length / 2, y);
    add(length / 2, y);
  }
}

std::vector<Cluster> clusters(const std::vector<RecentPoint> & points)
{
  ConnectedComponents c;
  MapParams none;
  c.initialize(none);
  std::vector<Cluster> out;
  c.cluster(points, out);
  return out;
}

const Track * byId(const Tracker & tracker, uint32_t id)
{
  for (const Track & t : tracker.tracks()) {
    if (t.id == id) {
      return &t;
    }
  }
  return nullptr;
}
}  // namespace

TEST(Objects, ClustersAndWhatTheyLookLike)
{
  std::vector<RecentPoint> points;
  box(points, Eigen::Vector2f(10.0f, 0.0f), 0.5f, 0.5f, 1.7f, 1.0);      // a pedestrian
  box(points, Eigen::Vector2f(10.0f, 3.0f), 4.4f, 1.8f, 1.5f, 1.0);      // a car
  box(points, Eigen::Vector2f(20.0f, -2.0f), 0.4f, 0.4f, 0.6f, 1.0);     // a cone
  box(points, Eigen::Vector2f(0.0f, 10.0f), 14.0f, 0.4f, 1.2f, 1.0);     // a wall
  const auto out = clusters(points);
  ASSERT_EQ(out.size(), 4u);
  int found = 0;
  for (const Cluster & c : out) {
    if ((c.centre - Eigen::Vector2f(10.0f, 0.0f)).norm() < 0.1f) {
      EXPECT_EQ(c.classification, ObjectClass::kPedestrian);
      ++found;
    } else if ((c.centre - Eigen::Vector2f(10.0f, 3.0f)).norm() < 0.1f) {
      EXPECT_EQ(c.classification, ObjectClass::kVehicle);
      EXPECT_NEAR(c.length, 4.4f, 0.05f);
      EXPECT_NEAR(c.width, 1.8f, 0.05f);
      EXPECT_NEAR(std::abs(std::cos(c.yaw)), 1.0f, 1e-3f);  // along x
      ++found;
    } else if ((c.centre - Eigen::Vector2f(20.0f, -2.0f)).norm() < 0.1f) {
      EXPECT_EQ(c.classification, ObjectClass::kSmall);
      ++found;
    } else if ((c.centre - Eigen::Vector2f(0.0f, 10.0f)).norm() < 0.1f) {
      EXPECT_EQ(c.classification, ObjectClass::kStructure);
      ++found;
    }
  }
  EXPECT_EQ(found, 4);
}

TEST(Objects, AWalkingPedestriansSpeed)
{
  // Crossing the road at 1.4 m/s, a 10 Hz lidar, the grid at 20 Hz (each scan twice)
  KalmanTracker tracker;
  MapParams none;
  tracker.initialize(none);
  std::mt19937 rng(1);
  uint32_t id = 0;
  for (int k = 0; k <= 40; ++k) {
    const double t = 0.05 * k;
    const double scan = 0.1 * std::floor(t / 0.1 + 1e-9);
    std::vector<RecentPoint> points;
    box(points, Eigen::Vector2f(15.0f, -3.0f + 1.4f * static_cast<float>(scan)), 0.5f, 0.4f, 1.75f, scan, &rng, true);
    tracker.update(clusters(points), t);
    ASSERT_EQ(tracker.tracks().size(), 1u) << t;  // the same scan again starts no track
    if (id == 0) {
      id = tracker.tracks()[0].id;
    }
    EXPECT_EQ(tracker.tracks()[0].id, id);
  }
  const Track & track = tracker.tracks()[0];
  EXPECT_TRUE(track.confirmed);
  EXPECT_TRUE(track.moving);
  EXPECT_NEAR(track.x(2), 0.0f, 0.2f);
  EXPECT_NEAR(track.x(3), 1.4f, 0.2f);
  EXPECT_EQ(track.last.classification, ObjectClass::kPedestrian);
}

TEST(Objects, AStandingBoxDoesNotMove)
{
  KalmanTracker tracker;
  MapParams none;
  tracker.initialize(none);
  std::mt19937 rng(2);
  for (int k = 0; k <= 50; ++k) {
    std::vector<RecentPoint> points;
    box(points, Eigen::Vector2f(12.0f, 2.0f), 0.6f, 0.6f, 0.9f, 0.1 * k, &rng);
    tracker.update(clusters(points), 0.1 * k);
    for (const Track & t : tracker.tracks()) {
      EXPECT_FALSE(t.moving) << 0.1 * k;
    }
  }
  EXPECT_EQ(tracker.tracks().size(), 1u);
}

TEST(Objects, TwoPedestriansPassingKeepTheirIds)
{
  // Walking towards each other 1.0 m apart sideways, at 1.4 m/s each
  KalmanTracker tracker;
  MapParams none;
  tracker.initialize(none);
  std::mt19937 rng(3);
  uint32_t a = 0, b = 0;
  for (int k = 0; k <= 50; ++k) {
    const double t = 0.1 * k;
    std::vector<RecentPoint> points;
    box(points, Eigen::Vector2f(10.0f + 1.4f * static_cast<float>(t), 0.0f), 0.5f, 0.4f, 1.7f, t, &rng, true);
    box(points, Eigen::Vector2f(17.0f - 1.4f * static_cast<float>(t), 1.0f), 0.5f, 0.4f, 1.7f, t, &rng, true);
    tracker.update(clusters(points), t);
    if (k == 0) {
      ASSERT_EQ(tracker.tracks().size(), 2u);
      a = tracker.tracks()[0].x(1) < 0.5f ? tracker.tracks()[0].id : tracker.tracks()[1].id;
      b = tracker.tracks()[0].x(1) < 0.5f ? tracker.tracks()[1].id : tracker.tracks()[0].id;
    }
  }
  const Track * ta = byId(tracker, a);
  const Track * tb = byId(tracker, b);
  ASSERT_TRUE(ta && tb);
  EXPECT_NEAR(ta->x(1), 0.0f, 0.2f);  // still the one on y = 0
  EXPECT_NEAR(ta->x(2), 1.4f, 0.25f);
  EXPECT_NEAR(tb->x(1), 1.0f, 0.2f);
  EXPECT_NEAR(tb->x(2), -1.4f, 0.25f);
}

TEST(Objects, WhatIsNotSeenGoes)
{
  KalmanTracker tracker;
  MapParams none;
  tracker.initialize(none);
  for (int k = 0; k < 5; ++k) {
    std::vector<RecentPoint> points;
    box(points, Eigen::Vector2f(8.0f, 0.0f), 0.6f, 0.6f, 0.9f, 0.1 * k);
    tracker.update(clusters(points), 0.1 * k);
  }
  ASSERT_EQ(tracker.tracks().size(), 1u);
  tracker.update({}, 0.8);   // 0.4 s unseen: still there
  EXPECT_EQ(tracker.tracks().size(), 1u);
  tracker.update({}, 1.0);   // 0.6 s: gone
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST(Objects, WhatTakesNoFreeCellDoesNotMove)
{
  // A barrier cut by one lidar ring: a short line sliding along it at the car's 8 m/s as the
  // car drives; its cells were never seen free (the rays end on the barrier)
  KalmanTracker tracker;
  MapParams none;
  tracker.initialize(none);
  for (int k = 0; k <= 30; ++k) {
    const double t = 0.1 * k;
    std::vector<RecentPoint> points;
    for (float u = 0.0f; u <= 0.8f; u += 0.2f) {
      points.push_back({Eigen::Vector2f(20.0f + 8.0f * static_cast<float>(t) + u, 6.0f), 1.2f, t, kNever});
    }
    tracker.update(clusters(points), t);
    for (const Track & track : tracker.tracks()) {
      EXPECT_FALSE(track.moving) << t;
    }
  }
  // A pedestrian walking where nothing was seen free (out of the rays' reach): not moving
  // either; the free cells are the evidence
  KalmanTracker blind;
  blind.initialize(none);
  for (int k = 0; k <= 30; ++k) {
    std::vector<RecentPoint> points;
    box(points, Eigen::Vector2f(15.0f, -3.0f + 0.14f * k), 0.5f, 0.4f, 1.75f, 0.1 * k);
    blind.update(clusters(points), 0.1 * k);
  }
  ASSERT_EQ(blind.tracks().size(), 1u);
  EXPECT_NEAR(blind.tracks()[0].x(3), 1.4f, 0.2f);  // its speed is known
  EXPECT_FALSE(blind.tracks()[0].moving);
}

TEST(Objects, OnceMovingItStaysMovingForAWhileWithoutFreshCells)
{
  // A person crossing: moving on fresh cells, then the cells it takes are not seen free (out
  // of the rays' reach): it keeps moving for 1.5 s after its last fresh cells, then no more
  KalmanTracker tracker;
  MapParams none;
  tracker.initialize(none);
  for (int k = 0; k <= 40; ++k) {
    const double t = 0.1 * k;
    std::vector<RecentPoint> points;
    box(points, Eigen::Vector2f(15.0f, -3.0f + 1.4f * static_cast<float>(t)), 0.5f, 0.4f, 1.75f, t, nullptr,
      k < 15);
    tracker.update(clusters(points), t);
    if (k >= 10) {
      ASSERT_EQ(tracker.tracks().size(), 1u);
      if (k <= 28) {
        EXPECT_TRUE(tracker.tracks()[0].moving) << t;  // the last fresh cells at 1.4 s
      } else if (k >= 30) {
        EXPECT_FALSE(tracker.tracks()[0].moving) << t;
      }
    }
  }
}

TEST(Objects, ACarSeenFromACornerGetsItsSides)
{
  // Two sides of a 4.4 x 1.8 m car turned 30 degrees, as seen from one corner (an L): the
  // points' main direction is between its sides; the L-shape fit finds them
  const float yaw = 30.0f * static_cast<float>(M_PI) / 180.0f;
  const Eigen::Vector2f u(std::cos(yaw), std::sin(yaw)), v(-u.y(), u.x());
  const Eigen::Vector2f corner(12.0f, 3.0f);
  std::vector<RecentPoint> points;
  for (float s = 0.0f; s <= 4.4f + 1e-3f; s += 0.2f) {
    points.push_back({corner + s * u, 1.5f, 1.0, kNever});
  }
  for (float s = 0.2f; s <= 1.8f + 1e-3f; s += 0.2f) {
    points.push_back({corner + s * v, 1.5f, 1.0, kNever});
  }
  const auto out = clusters(points);
  ASSERT_EQ(out.size(), 1u);
  const Cluster & c = out[0];
  EXPECT_EQ(c.classification, ObjectClass::kVehicle);
  EXPECT_EQ(c.shape, ObjectShape::kBox);
  EXPECT_NEAR(std::remainder(c.yaw - yaw, static_cast<float>(M_PI)), 0.0f, 0.03f);
  EXPECT_NEAR(c.length, 4.4f, 0.05f);
  EXPECT_NEAR(c.width, 1.8f, 0.05f);
  EXPECT_LT((c.centre - (corner + 2.2f * u + 0.9f * v)).norm(), 0.05f);
}

TEST(Objects, ACurvedWallIsDrawnAlongItsBend)
{
  // A wall on a 40 m bend, 30 m of it: one box would cover the road inside the bend
  std::vector<RecentPoint> points;
  const float radius = 40.0f;
  for (float a = 0.0f; a <= 30.0f / radius; a += 0.2f / radius) {
    points.push_back({Eigen::Vector2f(radius * std::cos(a), radius * std::sin(a)), 1.0f, 1.0, kNever});
  }
  const auto out = clusters(points);
  ASSERT_EQ(out.size(), 1u);
  const Cluster & c = out[0];
  EXPECT_EQ(c.classification, ObjectClass::kStructure);
  EXPECT_EQ(c.shape, ObjectShape::kPolygons);
  EXPECT_GE(c.footprint.size(), 10u);
  // The road 2 m inside the bend, at its middle, is in none of the pieces
  const float mid = 15.0f / radius;
  const Eigen::Vector2f road((radius - 2.0f) * std::cos(mid), (radius - 2.0f) * std::sin(mid));
  for (const auto & polygon : c.footprint) {
    for (const auto & q : polygon) {
      EXPECT_GT((q - road).norm(), 1.5f);
    }
  }
}

TEST(Objects, APersonIsACircle)
{
  // The near half of a 0.5 m wide person (what a lidar sees)
  std::vector<RecentPoint> points;
  for (float a = static_cast<float>(M_PI) / 2; a <= 3 * static_cast<float>(M_PI) / 2 + 1e-3f; a += 0.3f) {
    points.push_back({Eigen::Vector2f(10.0f + 0.25f * std::cos(a), 0.25f * std::sin(a)), 1.7f, 1.0, kNever});
  }
  const auto out = clusters(points);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].shape, ObjectShape::kCylinder);
  EXPECT_NEAR(out[0].length, 0.5f, 0.05f);           // its diameter
  EXPECT_LT((out[0].centre - Eigen::Vector2f(10.0f, 0.0f)).norm(), 0.26f);  // at most its radius off
}

TEST(Objects, WhatHasNoHeightDoesNotMove)
{
  // A 2D lidar's plane meeting the road ahead: a thin line, no height, sliding across cells
  // known free at 6 m/s
  KalmanTracker tracker;
  MapParams none;
  tracker.initialize(none);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (int k = 0; k <= 30; ++k) {
    const double t = 0.1 * k;
    std::vector<RecentPoint> points;
    for (float u = 0.0f; u <= 0.6f; u += 0.2f) {
      points.push_back({Eigen::Vector2f(12.0f + u, -4.0f + 6.0f * static_cast<float>(t)), nan, t, true});
    }
    tracker.update(clusters(points), t);
    for (const Track & track : tracker.tracks()) {
      EXPECT_FALSE(track.moving) << t;
    }
  }
}

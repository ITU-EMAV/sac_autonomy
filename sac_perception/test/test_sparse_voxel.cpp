// The 3D representations (sparse_voxel, multi_level_surface: every test runs for both) and
// the traversability rules: what is over the car is not an obstacle, what it would touch is, a
// ceiling lower than the car closes the way, a bridge's deck does not replace the road under
// it, and rays passing through an element clear it.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sac_perception/multi_level_surface.hpp"
#include "sac_perception/sparse_voxel.hpp"

using namespace sac_perception;

namespace
{
constexpr float kNoGround = -std::numeric_limits<float>::infinity();

VehicleBox car()
{
  VehicleBox box;
  box.min = Eigen::Vector3f(-1.4f, -0.9f, 0.0f);
  box.max = Eigen::Vector3f(1.45f, 0.9f, 1.83f);  // top of the band: 1.83 + 0.3
  box.known = true;
  return box;
}

template<typename Map>
struct Fixture
{
  Map map;
  int source = 0;
  double time = 100.0;

  explicit Fixture(MapParams params = {})
  {
    GridGeometry geometry;
    geometry.size = 60.0;
    map.initialize(params, geometry, car());
    LayerParams layer;
    layer.decay = 0.2f;
    source = map.addSource("lidar", layer);
    map.recenter(0.0, 0.0);
  }

  /// Flat ground at z over a square, as a ground filter's points would set it
  void ground(float z, float x0 = -29.0f, float x1 = 29.0f, float y0 = -29.0f, float y1 = 29.0f)
  {
    for (float x = x0; x <= x1; x += 0.2f) {
      for (float y = y0; y <= y1; y += 0.2f) {
        map.setGround(x + 0.1f, y + 0.1f, z, time);
      }
    }
  }

  void scan(const std::vector<Eigen::Vector3f> & hits, const std::vector<Eigen::Vector3f> & free = {})
  {
    Scan s;
    s.origin = Eigen::Vector3f(0.0f, 0.0f, 1.8f);
    s.time = time;
    for (const auto & p : hits) {
      Ray r;
      r.end = p;
      r.ground_z = kNoGround;
      s.rays.push_back(r);
    }
    for (const auto & p : free) {
      Ray r;
      r.end = p;
      r.hit = false;
      r.ground_z = p.z();
      s.rays.push_back(r);
    }
    map.insert(source, s);
    time += 0.1;
  }

  int value(double x, double y) const
  {
    int i = static_cast<int>(std::floor((x - map.originX()) / map.resolution()));
    int j = static_cast<int>(std::floor((y - map.originY()) / map.resolution()));
    return map.project()[j * map.width() + i];
  }
  bool blocked(double x, double y) const { return value(x, y) >= 65; }
};
template<typename Map>
class ColumnMaps : public testing::Test
{
};
using Maps = testing::Types<SparseVoxel, MultiLevelSurface>;
TYPED_TEST_SUITE(ColumnMaps, Maps);
}  // namespace

TYPED_TEST(ColumnMaps, WhatTheCarWouldTouchAndWhatIsOverIt)
{
  Fixture<TypeParam> f;
  f.ground(0.0f);
  for (int k = 0; k < 3; ++k) {
    f.scan({
      Eigen::Vector3f(10.05f, 0.05f, 1.9f),    // hanging down to 1.9 m: the roof would hit it
      Eigen::Vector3f(10.05f, 2.05f, 4.0f),    // a sign
      Eigen::Vector3f(15.05f, 0.05f, 8.0f),    // a bridge's deck
      Eigen::Vector3f(10.05f, -2.05f, 0.15f),  // a kerb
      Eigen::Vector3f(10.05f, -4.05f, 0.5f)}); // a box
  }
  EXPECT_TRUE(f.blocked(10.05, 0.05));
  EXPECT_FALSE(f.blocked(10.05, 2.05));
  EXPECT_FALSE(f.blocked(15.05, 0.05));
  EXPECT_FALSE(f.blocked(10.05, -2.05));
  EXPECT_TRUE(f.blocked(10.05, -4.05));
}

TYPED_TEST(ColumnMaps, ACeilingLowerThanTheCarClosesTheWay)
{
  Fixture<TypeParam> f;
  f.ground(0.0f);
  std::vector<Eigen::Vector3f> low, high;
  for (float x = 20.05f; x < 22.0f; x += 0.2f) {
    low.emplace_back(x, 3.05f, 1.8f);    // a tunnel 1.8 m high
    high.emplace_back(x, -3.05f, 3.0f);  // one 3 m high
  }
  low.insert(low.end(), high.begin(), high.end());
  for (int k = 0; k < 3; ++k) {
    f.scan(low);
  }
  EXPECT_TRUE(f.blocked(21.05, 3.05));
  EXPECT_FALSE(f.blocked(21.05, -3.05));
}

TYPED_TEST(ColumnMaps, ADeckDoesNotReplaceTheRoadUnderIt)
{
  Fixture<TypeParam> f;
  f.ground(0.0f);
  f.ground(8.0f, 14.0f, 18.0f, -4.0f, 4.0f);  // the deck's top, taken for ground
  EXPECT_NEAR(f.map.columnGround(16.05, 0.05), 0.0f, 1e-3f);
  // Its railings 1 m over the deck are 9 m over the road: over the car
  for (int k = 0; k < 3; ++k) {
    f.scan({Eigen::Vector3f(16.05f, 3.05f, 9.0f)});
  }
  EXPECT_FALSE(f.blocked(16.05, 3.05));
  // Where the road goes down for good, the old height is let go once it is old
  f.time += 5.0;
  f.ground(-2.0f, 14.0f, 18.0f, -4.0f, 4.0f);
  f.ground(6.0f, 14.0f, 18.0f, -4.0f, 4.0f);
  EXPECT_NEAR(f.map.columnGround(16.05, 0.05), -2.0f, 1e-3f);
}

TYPED_TEST(ColumnMaps, WithoutGroundAnObstacleStaysAnObstacle)
{
  Fixture<TypeParam> f;
  for (int k = 0; k < 3; ++k) {
    f.scan({Eigen::Vector3f(10.05f, 0.05f, 0.1f)});
  }
  EXPECT_TRUE(std::isnan(f.map.columnGround(10.05, 0.05)));
  EXPECT_TRUE(f.blocked(10.05, 0.05));
}

TYPED_TEST(ColumnMaps, RaysThroughAnElementClearIt)
{
  Fixture<TypeParam> f;
  f.ground(0.0f);
  for (int k = 0; k < 3; ++k) {
    f.scan({Eigen::Vector3f(10.05f, 0.05f, 0.5f)});
  }
  ASSERT_TRUE(f.blocked(10.05, 0.05));
  // The box has gone: rays to the ground behind it pass where it was, 0.4-0.6 m up
  std::vector<Eigen::Vector3f> ground;
  for (float x = 13.4f; x < 14.6f; x += 0.05f) {
    ground.emplace_back(x, 0.05f + 0.35f * (x - 13.4f) / 10.0f, 0.0f);
  }
  // A ray of the same scan that hits it does not clear it
  f.scan({Eigen::Vector3f(10.05f, 0.05f, 0.5f)}, ground);
  EXPECT_TRUE(f.blocked(10.05, 0.05));
  f.scan({}, ground);  // two dozen rays through it
  EXPECT_FALSE(f.blocked(10.05, 0.05));
  EXPECT_EQ(f.value(10.05, 0.05), 0);  // seen free
}

TYPED_TEST(ColumnMaps, WithoutMemoryOnlyTheLastScanCounts)
{
  MapParams params;
  params.doubles["memory"] = 0.0;
  Fixture<TypeParam> f(params);
  f.ground(0.0f);
  const int other = f.map.addSource("front", LayerParams{});
  f.scan({Eigen::Vector3f(10.05f, 0.05f, 0.5f)});
  EXPECT_TRUE(f.blocked(10.05, 0.05));  // one scan is enough
  Scan front;  // another sensor's scan does not take this one's voxels away
  front.origin = Eigen::Vector3f(1.5f, 0.0f, 0.45f);
  front.time = f.time;
  f.map.insert(other, front);
  EXPECT_TRUE(f.blocked(10.05, 0.05));
  f.scan({Eigen::Vector3f(12.05f, 0.05f, 0.5f)});  // it moved
  EXPECT_FALSE(f.blocked(10.05, 0.05));
  EXPECT_TRUE(f.blocked(12.05, 0.05));

  MapParams windowed;  // a window: the scans of the last 0.25 s
  windowed.doubles["memory"] = 0.0;
  windowed.doubles["window"] = 0.25;
  Fixture<TypeParam> g(windowed);
  g.ground(0.0f);
  g.scan({Eigen::Vector3f(10.05f, 0.05f, 0.5f)});
  g.scan({});
  g.scan({});
  EXPECT_TRUE(g.blocked(10.05, 0.05));  // 0.2 s old
  g.scan({});
  EXPECT_FALSE(g.blocked(10.05, 0.05));
}

TYPED_TEST(ColumnMaps, MovingTheWindowKeepsWhatItHolds)
{
  Fixture<TypeParam> f;
  f.ground(0.0f);
  for (int k = 0; k < 3; ++k) {
    f.scan({Eigen::Vector3f(10.05f, 5.05f, 1.0f)});
  }
  f.map.recenter(8.0, 3.0);
  EXPECT_TRUE(f.blocked(10.05, 5.05));
}

TYPED_TEST(ColumnMaps, StepsInTheGround)
{
  MapParams params;
  params.doubles["max_step"] = 0.3;
  Fixture<TypeParam> f(params);
  f.ground(0.0f, -5.0f, 12.0f, -5.0f, 5.0f);
  f.ground(0.5f, 12.2f, 20.0f, -5.0f, 5.0f);   // a 0.5 m step at x = 12.2
  for (float x = -5.0f; x <= 20.0f; x += 0.2f) {  // a 20 % slope elsewhere: 0.04 m per cell
    for (float y = -12.0f; y <= -6.0f; y += 0.2f) {
      f.map.setGround(x + 0.1f, y + 0.1f, 0.2f * x, f.time);
    }
  }
  EXPECT_TRUE(f.blocked(12.15, 0.05));
  EXPECT_TRUE(f.blocked(12.35, 0.05));
  EXPECT_FALSE(f.blocked(10.05, 0.05));
  EXPECT_FALSE(f.blocked(15.05, 0.05));
  EXPECT_FALSE(f.blocked(10.05, -8.95));
}

TEST(MultiLevelSurface, APoleIsOneIntervalADeckAnother)
{
  Fixture<MultiLevelSurface> f;
  f.ground(0.0f);
  std::vector<Eigen::Vector3f> points;
  for (float z = 0.3f; z <= 1.5f; z += 0.25f) {  // a pole's points, 0.25 m apart
    points.emplace_back(10.05f, 0.05f, z);
  }
  points.emplace_back(10.05f, 0.05f, 8.0f);  // a deck over it
  points.emplace_back(10.05f, 0.05f, 8.2f);
  f.scan(points);
  const auto levels = f.map.column(10.05, 0.05);
  ASSERT_EQ(levels.size(), 2u);
  EXPECT_NEAR(levels[0].first, 0.3f, 1e-4f);
  EXPECT_NEAR(levels[0].second, 1.3f, 1e-4f);
  EXPECT_NEAR(levels[1].first, 8.0f, 1e-4f);
  EXPECT_NEAR(levels[1].second, 8.2f, 1e-4f);
  // An interval of its own at 2.1 m, then the pole growing up to it: they merge
  f.scan({Eigen::Vector3f(10.05f, 0.05f, 2.1f), Eigen::Vector3f(10.05f, 0.05f, 1.55f),
    Eigen::Vector3f(10.05f, 0.05f, 1.82f)});
  const auto merged = f.map.column(10.05, 0.05);
  ASSERT_EQ(merged.size(), 2u);
  EXPECT_NEAR(merged[0].first, 0.3f, 1e-4f);
  EXPECT_NEAR(merged[0].second, 2.1f, 1e-4f);
}

TEST(MultiLevelSurface, AtMostMaxLevels)
{
  MapParams params;
  params.doubles["max_levels"] = 3;
  Fixture<MultiLevelSurface> f(params);
  f.scan({
    Eigen::Vector3f(10.05f, 0.05f, 0.5f), Eigen::Vector3f(10.05f, 0.05f, 2.0f),
    Eigen::Vector3f(10.05f, 0.05f, 2.9f), Eigen::Vector3f(10.05f, 0.05f, 6.0f)});
  const auto levels = f.map.column(10.05, 0.05);
  ASSERT_EQ(levels.size(), 3u);  // the two closest (2.0, 2.9) became one
  EXPECT_NEAR(levels[1].first, 2.0f, 1e-4f);
  EXPECT_NEAR(levels[1].second, 2.9f, 1e-4f);
}

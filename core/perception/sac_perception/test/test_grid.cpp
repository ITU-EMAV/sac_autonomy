// The rolling grid: rays mark and clear the right cells, the window moves without moving what
// it holds, and the distance transform is exact.

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "sac_perception/grid.hpp"
#include "sac_perception/map_representation.hpp"

using namespace sac_perception;

namespace
{
float at(const RollingGrid & g, int layer, double x, double y)
{
  int i = 0;
  int j = 0;
  EXPECT_TRUE(g.cell(x, y, i, j)) << x << ", " << y;
  return g.logOdds(layer, i, j);
}

Ray ray(float x, float y, float z, bool hit, float ground_z = -std::numeric_limits<float>::infinity())
{
  Ray r;
  r.end = Eigen::Vector3f(x, y, z);
  r.hit = hit;
  r.ground_z = ground_z;
  return r;
}
}  // namespace

TEST(Grid, A2dRayMarksItsEndAndClearsTheWay)
{
  RollingGrid g(40.0, 0.2);
  const int l = g.addLayer("scan", LayerParams{});
  g.recenter(0.0, 0.0);
  Scan scan;
  scan.origin = Eigen::Vector3f(0.0f, 0.0f, 0.5f);
  scan.rays = {ray(5.05f, 0.05f, 0.5f, true)};
  g.integrate(l, scan);
  EXPECT_GT(at(g, l, 5.05, 0.05), 0.0f);
  for (double x : {0.5, 2.0, 4.5}) {
    EXPECT_LT(at(g, l, x, 0.05), 0.0f) << x;
  }
  EXPECT_EQ(at(g, l, 7.0, 0.05), 0.0f);  // beyond: unknown
  EXPECT_EQ(at(g, l, 2.0, 3.0), 0.0f);   // beside: unknown
}

TEST(Grid, A3dRayClearsOnlyWhereItRunsLow)
{
  // A roof lidar at 1.8 m sees the ground 20 m ahead; the ray passes 0.9 m over a 0.5 m
  // obstacle at 10 m, which must not be cleared; the last metres of the ray are free
  RollingGrid g(60.0, 0.2);
  const int l = g.addLayer("lidar", LayerParams{});
  g.recenter(0.0, 0.0);
  Scan hit;
  hit.origin = Eigen::Vector3f(0.0f, 0.0f, 1.8f);
  hit.clear_height = 0.3f;
  hit.rays = {ray(10.05f, 0.05f, 0.5f, true, 0.0f)};
  g.integrate(l, hit);
  const float obstacle = at(g, l, 10.05, 0.05);
  Scan ground;
  ground.origin = hit.origin;
  ground.clear_height = 0.3f;
  ground.rays = {ray(20.05f, 0.05f, 0.0f, false, 0.0f)};
  g.integrate(l, ground);
  EXPECT_EQ(at(g, l, 10.05, 0.05), obstacle);
  EXPECT_EQ(at(g, l, 5.0, 0.05), 0.0f);
  // Runs lower than 0.3 m from 20 * (1 - 0.3 / 1.8) = 16.7 m on
  EXPECT_LT(at(g, l, 18.0, 0.05), 0.0f);
  EXPECT_LT(at(g, l, 20.05, 0.05), 0.0f);  // the ground point itself is free
  EXPECT_EQ(at(g, l, 15.0, 0.05), 0.0f);
}

TEST(Grid, AnUnmarkedHitClearsTheWayButNotItsEnd)
{
  RollingGrid g(40.0, 0.2);
  const int l = g.addLayer("scan", LayerParams{});
  g.recenter(0.0, 0.0);
  g.set(l, 12.05, 0.05, 2.0f);  // another sensor's obstacle there
  Scan scan;
  scan.origin = Eigen::Vector3f(0.0f, 0.0f, 0.5f);
  Ray r = ray(12.05f, 0.05f, 0.5f, true);
  r.mark = false;
  scan.rays = {r};
  g.integrate(l, scan);
  EXPECT_LT(at(g, l, 6.0, 0.05), 0.0f);               // free before it
  EXPECT_FLOAT_EQ(at(g, l, 12.05, 0.05), 2.0f);       // its end left alone
}

TEST(Grid, AHitIsNotClearedByItsOwnScan)
{
  RollingGrid g(40.0, 0.2);
  const int l = g.addLayer("scan", LayerParams{});
  g.recenter(0.0, 0.0);
  Scan scan;
  scan.origin = Eigen::Vector3f(0.0f, 0.0f, 0.5f);
  // A pole at 5 m, and a ray of the same scan passing through its cell to a wall at 10 m
  scan.rays = {ray(5.05f, 0.05f, 0.5f, true), ray(10.05f, 0.09f, 0.5f, true)};
  g.integrate(l, scan);
  EXPECT_GT(at(g, l, 5.05, 0.05), 0.0f);
}

TEST(Grid, MovingTheWindowKeepsWhatItHolds)
{
  RollingGrid g(40.0, 0.2);
  const int l = g.addLayer("scan", LayerParams{});
  g.recenter(0.0, 0.0);
  g.set(l, 8.1, -3.1, 2.0f);
  g.recenter(6.3, 4.7);  // moves by whole cells
  EXPECT_FLOAT_EQ(at(g, l, 8.1, -3.1), 2.0f);
  EXPECT_NEAR(g.originX() + g.width() * g.resolution() / 2.0, 6.3, g.resolution());
  g.recenter(100.0, 0.0);  // out of the window: forgotten
  g.recenter(6.3, 4.7);
  EXPECT_FLOAT_EQ(at(g, l, 8.1, -3.1), 0.0f);
}

TEST(Grid, LayersCombineToTheMostOccupied)
{
  RollingGrid g(10.0, 0.5);
  const int a = g.addLayer("a", LayerParams{});
  const int b = g.addLayer("b", LayerParams{});
  g.recenter(0.0, 0.0);
  g.set(a, 1.0, 1.0, -2.0f);  // a: free
  g.set(b, 1.0, 1.0, 3.0f);   // b: occupied
  g.set(a, 2.0, 2.0, -2.0f);
  int i = 0;
  int j = 0;
  const auto data = g.combined();
  g.cell(1.0, 1.0, i, j);
  EXPECT_GT(data[g.index(i, j)], 90);
  g.cell(2.0, 2.0, i, j);
  EXPECT_LT(data[g.index(i, j)], 15);
  g.cell(-3.0, -3.0, i, j);
  EXPECT_EQ(data[g.index(i, j)], -1);
  const auto occupied = g.occupied();
  g.cell(1.0, 1.0, i, j);
  EXPECT_EQ(occupied[g.index(i, j)], 1);
}

TEST(Grid, DecayFadesTowardsUnknown)
{
  RollingGrid g(10.0, 0.5);
  LayerParams p;
  p.decay = 1.0f;
  const int l = g.addLayer("a", p);
  g.recenter(0.0, 0.0);
  g.set(l, 1.0, 1.0, 3.0f);
  g.decay(1.0);
  EXPECT_NEAR(at(g, l, 1.0, 1.0), 3.0f * std::exp(-1.0f), 1e-5f);
}

TEST(Grid, GroundMapGivesTheNearestRecentHeight)
{
  RollingGrid g(40.0, 0.2);
  g.addLayer("a", LayerParams{});
  g.recenter(0.0, 0.0);
  g.setGround(5.0, 0.0, 0.40f, 10.0);
  g.setGround(5.6, 0.0, 0.46f, 10.0);
  float z = 0.0f;
  ASSERT_TRUE(g.groundNear(5.1, 0.1, 1.0, 11.0, 3.0, z));
  EXPECT_FLOAT_EQ(z, 0.40f);       // the nearest
  EXPECT_FALSE(g.groundNear(9.0, 0.0, 1.0, 11.0, 3.0, z));  // none within 1 m
  EXPECT_FALSE(g.groundNear(5.0, 0.0, 1.0, 14.0, 3.0, z));  // too old
  g.recenter(8.0, 3.0);            // moves with the window
  ASSERT_TRUE(g.groundNear(5.0, 0.0, 0.3, 11.0, 3.0, z));
  EXPECT_FLOAT_EQ(z, 0.40f);
}

TEST(Grid, DistanceTransformIsExact)
{
  const int w = 60;
  std::mt19937 random(5);
  std::bernoulli_distribution obstacle(0.01);
  std::vector<uint8_t> occupied(w * w, 0);
  for (auto & c : occupied) {
    c = obstacle(random) ? 1 : 0;
  }
  const auto d = distanceTransform(occupied, w, 0.2);
  for (int j = 0; j < w; ++j) {
    for (int i = 0; i < w; ++i) {
      double best = std::numeric_limits<double>::infinity();
      for (int q = 0; q < w; ++q) {
        for (int p = 0; p < w; ++p) {
          if (occupied[q * w + p]) {
            best = std::min(best, std::hypot(p - i, q - j) * 0.2);
          }
        }
      }
      ASSERT_NEAR(d[j * w + i], best, 1e-4) << i << ", " << j;
    }
  }
  EXPECT_TRUE(std::isinf(distanceTransform(std::vector<uint8_t>(w * w, 0), w, 0.2)[0]));
}

TEST(Grid, DirectProjectionIsTheGridOfStageOne)
{
  GridGeometry geometry;
  geometry.size = 40.0;
  DirectProjection map;
  MapParams none;
  map.initialize(none, geometry, VehicleBox{});
  RollingGrid g(geometry.size, geometry.resolution);
  g.setRecenterDistance(geometry.recenter_distance);
  const int a = map.addSource("a", LayerParams{});
  const int b = map.addSource("b", LayerParams{});
  g.addLayer("a", LayerParams{});
  g.addLayer("b", LayerParams{});
  map.recenter(0.0, 0.0);
  g.recenter(0.0, 0.0);
  Scan scan;
  scan.origin = Eigen::Vector3f(0.0f, 0.0f, 1.8f);
  std::mt19937 rng(3);
  std::uniform_real_distribution<float> xy(-15.0f, 15.0f);
  for (int k = 0; k < 500; ++k) {
    scan.rays.push_back(ray(xy(rng), xy(rng), 0.5f, k % 3 == 0, 0.0f));
  }
  scan.clear_height = 0.3f;
  map.insert(a, scan);
  g.integrate(0, scan);
  map.set(b, 3.0, 3.0, 2.0f);
  g.set(1, 3.0, 3.0, 2.0f);
  map.decay(0.1);
  g.decay(0.1);
  EXPECT_EQ(map.project(), g.combined());
  map.recenter(9.0, -7.0);
  g.recenter(9.0, -7.0);
  EXPECT_EQ(map.project(), g.combined());
  EXPECT_DOUBLE_EQ(map.originX(), g.originX());
}

TEST(Grid, TheNearestGroundRingByRingIsTheNearest)
{
  // Ground in scattered cells; the ring search finds a cell as near as any (brute force)
  RollingGrid g(40.0, 0.2);
  g.recenter(0.0, 0.0);
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> xy(-19.0f, 19.0f);
  std::vector<Eigen::Vector2f> cells;
  for (int k = 0; k < 300; ++k) {
    const Eigen::Vector2f p(xy(rng), xy(rng));
    g.setGround(p.x(), p.y(), 0.01f * k, 10.0);
    cells.push_back(p);
  }
  for (int k = 0; k < 2000; ++k) {
    const double x = xy(rng);
    const double y = xy(rng);
    int ci = 0, cj = 0;
    g.cell(x, y, ci, cj);
    int best = std::numeric_limits<int>::max();
    for (const auto & p : cells) {
      int i = 0, j = 0;
      g.cell(p.x(), p.y(), i, j);
      const int d2 = (i - ci) * (i - ci) + (j - cj) * (j - cj);
      if (d2 <= 10 * 10) {
        best = std::min(best, d2);
      }
    }
    float z = 0.0f;
    const bool found = g.groundNear(x, y, 2.0, 10.0, 3.0, z);
    ASSERT_EQ(found, best != std::numeric_limits<int>::max()) << x << ", " << y;
    if (found) {
      // the cell it took is as near as the nearest
      int nearest = std::numeric_limits<int>::max();
      for (std::size_t c = 0; c < cells.size(); ++c) {
        int i = 0, j = 0;
        g.cell(cells[c].x(), cells[c].y(), i, j);
        const int d2 = (i - ci) * (i - ci) + (j - cj) * (j - cj);
        float zc = 0.0f;
        g.groundNear(cells[c].x(), cells[c].y(), 0.0, 10.0, 3.0, zc);
        if (std::abs(zc - z) < 1e-6f) {
          nearest = std::min(nearest, d2);
        }
      }
      EXPECT_EQ(nearest, best) << x << ", " << y;
    }
  }
}

TEST(Grid, WhatTakesCellsKnownFreeIsMoving)
{
  // A road seen free scan after scan; a person stepping onto it is moving, a wall the rays
  // always end on is not, and something that comes to stay stops being so
  RollingGrid g(40.0, 0.2);
  const int l = g.addLayer("lidar", LayerParams{});
  g.recenter(0.0, 0.0);
  double t = 10.0;
  auto scan = [&](std::vector<Ray> rays) {
    Scan s;
    s.origin = Eigen::Vector3f(0.0f, 0.0f, 1.8f);
    s.time = t;
    s.clear_height = 0.3f;
    s.rays = std::move(rays);
    g.integrate(l, s);
    t += 0.1;
  };
  std::vector<Ray> road;
  for (float x : {11.65f, 12.05f, 12.45f}) {  // ground returns across the road, three cells deep
    for (float y = -3.0f; y <= 3.0f; y += 0.1f) {
      road.push_back(ray(x, y, 0.0f, false, 0.0f));
    }
  }
  std::vector<Ray> with_wall = road;
  with_wall.push_back(ray(12.05f, 6.05f, 1.0f, true, 0.0f));  // a wall beside it
  for (int k = 0; k < 10; ++k) {
    scan(with_wall);
  }
  EXPECT_TRUE(g.dynamicAt(12.05, 0.05));   // the road: known free
  EXPECT_FALSE(g.dynamicAt(12.05, 6.05));  // the wall: never free
  std::vector<Ray> person = road;
  person.push_back(ray(12.05f, 0.05f, 1.0f, true, 0.0f));
  scan(person);
  EXPECT_TRUE(g.dynamicAt(12.05, 0.05));   // it took a free cell: moving
  for (int k = 0; k < 12; ++k) {           // it stays 1.2 s: it came to stay
    scan(person);
  }
  EXPECT_FALSE(g.dynamicAt(12.05, 0.05));
}

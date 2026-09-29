// The rolling grid: rays mark and clear the right cells, the window moves without moving what
// it holds, and the distance transform is exact.

#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "sac_perception/grid.hpp"

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

// The point filters on synthetic clouds: the roof lidar's rings on slopes and crests, a box and a wall.

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <cstdlib>
#include <map>
#include <random>

#include "sac_perception/filters.hpp"
#include "sac_perception/ground_patchwork.hpp"

using namespace sac_perception;

namespace
{
/// Ground points around the car (as a lidar's rings would give them, denser near) on the
/// surface z = ground(x, y), and a 1 x 1 x 1 m box standing on it at (bx, by). Returns the
/// number of ground points (they come first).
std::size_t scene(Cloud & cloud, const std::function<float(float, float)> & ground, float bx, float by)
{
  std::mt19937 random(11);
  std::normal_distribution<float> noise(0.0f, 0.015f);
  std::vector<Eigen::Vector3f> points;
  for (float r = 3.0f; r < 58.0f; r *= 1.06f) {
    const int n = 360;
    for (int k = 0; k < n; ++k) {
      const float a = k * 6.2831853f / n;
      const float x = -0.45f + r * std::cos(a);
      const float y = r * std::sin(a);
      if (std::abs(x - bx) < 0.6f && std::abs(y - by) < 0.6f) {
        continue;  // under the box
      }
      points.emplace_back(x, y, ground(x, y) + noise(random));
    }
  }
  const std::size_t ground_points = points.size();
  const float base = ground(bx, by);
  for (float u = -0.5f; u <= 0.5f; u += 0.1f) {
    for (float h = 0.05f; h <= 1.0f; h += 0.1f) {
      points.emplace_back(bx - 0.5f, by + u, base + h);  // the face towards the car
      points.emplace_back(bx + u, by - 0.5f, base + h);
    }
  }
  cloud.resize(points.size());
  cloud.points = points;
  cloud.origin = Eigen::Vector3f(-0.45f, 0.0f, 1.79f);
  return ground_points;
}

/// The same with the ground as a VLP-16 on the roof sees it: only the rings where its 8
/// downward channels (-15..-1 deg, 2 deg apart) meet the surface, 0.4 deg apart. A bin then
/// often holds one ring, a line.
std::size_t lidarScene(Cloud & cloud, const std::function<float(float, float)> & ground, float bx, float by)
{
  const Eigen::Vector3f origin(-0.45f, 0.0f, 1.79f);
  std::vector<Eigen::Vector3f> points;
  // The simulated VLP-16's range noise (without any, every bin is alike and the learnt
  // thresholds collapse onto their mean)
  static std::mt19937 random(17);
  std::normal_distribution<float> noise(0.0f, 0.015f);
  for (int channel = -15; channel <= -1; channel += 2) {
    const float elevation = channel * 3.14159265f / 180.0f;
    for (float azimuth = 0.0f; azimuth < 360.0f; azimuth += 0.4f) {
      const float a = azimuth * 3.14159265f / 180.0f;
      const Eigen::Vector3f direction(
        std::cos(elevation) * std::cos(a), std::cos(elevation) * std::sin(a), std::sin(elevation));
      for (float r = 1.0f; r < 60.0f; r += 0.05f) {
        const Eigen::Vector3f p = origin + r * direction;
        if (std::abs(p.x() - bx) < 0.6f && std::abs(p.y() - by) < 0.6f) {
          break;  // behind the box
        }
        if (p.z() <= ground(p.x(), p.y())) {
          points.push_back(origin + (r + noise(random)) * direction);
          break;
        }
      }
    }
  }
  const std::size_t ground_points = points.size();
  const float base = ground(bx, by);
  for (float u = -0.5f; u <= 0.5f; u += 0.1f) {
    for (float h = 0.05f; h <= 1.0f; h += 0.1f) {
      points.emplace_back(bx - 0.5f, by + u, base + h);
    }
  }
  cloud.resize(points.size());
  cloud.points = points;
  cloud.origin = origin;
  return ground_points;
}

struct Counts
{
  double ground_as_ground;
  double box_as_obstacle;
  float ground_z_error;  // at the box
};

/// Runs `scans` scans of the scene through the filter (a learning filter sees the same
/// scene again, as a car standing there would) and counts the last one
Counts classify(
  PointFilter & filter, const std::function<float(float, float)> & ground, float bx, float by,
  bool lidar = false, int scans = 1)
{
  Cloud cloud;
  std::size_t n_ground = 0;
  for (int s = 0; s < scans; ++s) {
    n_ground = lidar ? lidarScene(cloud, ground, bx, by) : scene(cloud, ground, bx, by);
    filter.apply(cloud);
  }
  std::size_t good_ground = 0;
  std::size_t good_box = 0;
  std::size_t box = 0;
  float error = 0.0f;
  std::map<int, int> wrong;  // by range [m / 5]
  for (std::size_t k = 0; k < cloud.size(); ++k) {
    if (k < n_ground) {
      good_ground += cloud.labels[k] == Cloud::kGround;
      if (cloud.labels[k] != Cloud::kGround) {
        ++wrong[static_cast<int>(std::hypot(cloud.points[k].x() + 0.45f, cloud.points[k].y()) / 5.0f)];
      }
      continue;
    }
    // The box's points more than 0.2 m up (lower ones are within the ground's tolerance, and
    // height_band drops them anyway)
    const Eigen::Vector3f & p = cloud.points[k];
    if (p.z() - ground(p.x(), p.y()) < 0.2f) {
      continue;
    }
    ++box;
    good_box += cloud.labels[k] == Cloud::kObstacle;
    error = std::max(error, std::abs(cloud.ground_z[k] - ground(p.x(), p.y())));
  }
  if (std::getenv("DEBUG_GROUND")) {
    for (const auto & [bucket, count] : wrong) {
      std::printf("  wrong ground %2d-%2d m: %d\n", bucket * 5, bucket * 5 + 5, count);
    }
  }
  return {static_cast<double>(good_ground) / n_ground, static_cast<double>(good_box) / box, error};
}

/// Patchwork++'s defaults with the VLP-16's num_min_pts (config/sim.yaml); any parameter can
/// be overridden from the environment for tuning, e.g. th_dist=0.2
GroundPatchwork patchwork()
{
  GroundPatchwork f;
  MapParams p;
  p.doubles["num_min_pts"] = 3;
  for (const char * key : {"th_seeds", "th_dist", "num_lpr", "num_min_pts"}) {
    if (const char * v = std::getenv(key)) {
      p.doubles[key] = std::atof(v);
    }
  }
  f.initialize(p);
  return f;
}
}  // namespace

TEST(GroundPatchwork, FindsTheGroundInALidarsRings)
{
  // The roof VLP-16's rings on flat ground, slopes, a crest, and with the car pitched against
  // the road (braking); a 1 m box 18 m ahead. The filter learns its thresholds (A-GLE) over
  // the scans, as it does while driving.
  struct Case
  {
    const char * name;
    std::function<float(float, float)> ground;
  };
  for (const Case & c : std::vector<Case>{
      {"flat", [](float, float) { return 0.0f; }},
      {"uphill from 5 m", [](float x, float y) { return x > 5.0f ? 0.08f * (x - 5.0f) + 0.01f * y : 0.01f * y; }},
      {"crest", [](float x, float) { return x > 10.0f ? -0.1f * (x - 10.0f) : 0.0f; }},
      {"pitched 2 deg", [](float x, float) { return 0.035f * x; }}})
  {
    GroundPatchwork f = patchwork();
    const Counts r = classify(f, c.ground, 18.0f, 1.0f, true, 5);
    if (std::getenv("DEBUG_GROUND")) {
      const auto & s = f.statistics();
      std::printf("%s: sparse %zu no_plane %zu not_upright %zu heading %zu elevated %zu off_plane %zu reverted %zu\n", c.name, s.sparse, s.no_plane, s.not_upright, s.heading, s.elevated, s.off_plane, s.reverted);
    }
    EXPECT_GT(r.ground_as_ground, 0.99) << c.name;
    EXPECT_GT(r.box_as_obstacle, 0.95) << c.name;
    EXPECT_LT(r.ground_z_error, 0.2f) << c.name;
  }
}

TEST(GroundPatchwork, FirstScanAlready)
{
  // Before it has learnt anything (thresholds 0), the ground is found
  GroundPatchwork f = patchwork();
  const Counts r = classify(f, [](float, float) { return 0.0f; }, 18.0f, 1.0f, true, 1);
  EXPECT_GT(r.ground_as_ground, 0.99);
  EXPECT_GT(r.box_as_obstacle, 0.95);
}

TEST(GroundPatchwork, LearnsTheSensorHeight)
{
  GroundPatchwork f = patchwork();
  classify(f, [](float, float) { return 0.0f; }, 18.0f, 1.0f, true, 5);
  EXPECT_NEAR(f.sensorHeight(), 1.79, 0.05);  // the lidar's height over the ground
}

TEST(GroundPatchwork, AWallNearTheCarIsNotGround)
{
  // R-VPF: a 2 m wall 4 m to the left, 3 to 9 m ahead (zone 0), densely seen
  GroundPatchwork f = patchwork();
  Cloud cloud;
  for (int s = 0; s < 3; ++s) {
    const std::size_t n_ground = lidarScene(cloud, [](float, float) { return 0.0f; }, 30.0f, 30.0f);
    std::vector<Eigen::Vector3f> points(cloud.points.begin(), cloud.points.begin() + n_ground);
    const std::size_t wall_start = points.size();
    for (float x = 3.0f; x <= 9.0f; x += 0.1f) {
      for (float z = 0.0f; z <= 2.0f; z += 0.1f) {
        points.emplace_back(x, 4.0f, z);
      }
    }
    cloud.resize(points.size());
    cloud.points = points;
    f.apply(cloud);
    if (s == 2) {
      std::size_t wall = 0;
      std::size_t wall_up = 0;
      for (std::size_t k = wall_start; k < cloud.size(); ++k) {
        if (cloud.points[k].z() > 0.2f) {
          ++wall;
          wall_up += cloud.labels[k] == Cloud::kObstacle;
        }
      }
      EXPECT_GT(static_cast<double>(wall_up) / wall, 0.95);
    }
  }
}

TEST(GroundPatchwork, ABridgeOverTheRoadIsNeitherGroundNorAnObstacle)
{
  // The simulation's failure: a bridge deck 8 m over a flat road, 25-30 m ahead, 20 m wide
  // with 1 m railings; the roof VLP-16's 16 channels. Far out Patchwork++ took the flat
  // deck for ground and its railings for obstacles standing on it.
  const Eigen::Vector3f origin(-0.45f, 0.0f, 1.79f);
  auto deck = [](const Eigen::Vector3f & p) {
    return p.x() > 25.0f && p.x() < 30.0f && std::abs(p.y()) < 10.0f &&
           ((p.z() > 8.0f && p.z() < 8.5f) ||                                           // the deck
            (p.z() >= 8.5f && p.z() < 9.5f && (p.x() < 25.3f || p.x() > 29.7f)));      // railings
  };
  std::mt19937 random(5);
  std::normal_distribution<float> noise(0.0f, 0.015f);
  std::vector<Eigen::Vector3f> points;
  std::vector<bool> on_deck;
  for (int channel = -15; channel <= 15; channel += 2) {
    const float elevation = channel * 3.14159265f / 180.0f;
    for (float azimuth = 0.0f; azimuth < 360.0f; azimuth += 0.4f) {
      const float a = azimuth * 3.14159265f / 180.0f;
      const Eigen::Vector3f direction(
        std::cos(elevation) * std::cos(a), std::cos(elevation) * std::sin(a), std::sin(elevation));
      for (float r = 1.0f; r < 60.0f; r += 0.05f) {
        const Eigen::Vector3f p = origin + r * direction;
        if (p.z() <= 0.0f || deck(p)) {
          points.push_back(origin + (r + noise(random)) * direction);
          on_deck.push_back(p.z() > 1.0f);
          break;
        }
      }
    }
  }
  GroundPatchwork f = patchwork();
  HeightBand band;
  MapParams none;
  band.initialize(none);
  Cloud cloud;
  for (int scan = 0; scan < 3; ++scan) {
    cloud.resize(points.size());
    cloud.points = points;
    cloud.origin = origin;
    f.apply(cloud);
    band.apply(cloud);
  }
  std::size_t deck_points = 0;
  std::size_t deck_ground = 0;
  std::size_t deck_obstacle = 0;
  std::size_t road = 0;
  std::size_t road_ground = 0;
  for (std::size_t k = 0; k < cloud.size(); ++k) {
    if (on_deck[k]) {
      ++deck_points;
      deck_ground += cloud.labels[k] == Cloud::kGround;
      deck_obstacle += cloud.labels[k] == Cloud::kObstacle;
    } else {
      ++road;
      road_ground += cloud.labels[k] == Cloud::kGround;
    }
  }
  ASSERT_GT(deck_points, 50u);
  EXPECT_EQ(deck_ground, 0u);    // the deck is not the ground
  EXPECT_EQ(deck_obstacle, 0u);  // nor anything on it an obstacle: all too high to matter
  EXPECT_GT(static_cast<double>(road_ground) / road, 0.99);
}

TEST(GroundHeight, IsOnlyForFlatGround)
{
  GroundHeight f;
  MapParams p;
  f.initialize(p);
  const Counts flat = classify(f, [](float, float) { return 0.0f; }, 10.0f, 0.0f);
  EXPECT_GT(flat.ground_as_ground, 0.99);
  EXPECT_GT(flat.box_as_obstacle, 0.95);
  // On the slope the far ground is "obstacle": what ground_patchwork is for
  const Counts slope = classify(f, [](float x, float) { return 0.08f * x; }, 10.0f, 0.0f);
  EXPECT_LT(slope.ground_as_ground, 0.8);
}

TEST(Filters, CropBoxHeightBandAndVoxel)
{
  Cloud cloud;
  cloud.resize(5);
  cloud.points = {
    Eigen::Vector3f(0.5f, 0.2f, 1.0f),   // on the car
    Eigen::Vector3f(8.0f, 0.0f, 0.1f),   // a kerb
    Eigen::Vector3f(8.0f, 1.0f, 1.0f),   // an obstacle
    Eigen::Vector3f(8.0f, 2.0f, 4.0f),   // a branch overhead
    Eigen::Vector3f(8.01f, 1.01f, 1.01f)};  // next to the obstacle point
  CropBox crop;
  MapParams none;
  crop.initialize(none);
  HeightBand band;
  band.initialize(none);
  Voxel voxel;
  voxel.initialize(none);
  crop.apply(cloud);
  band.apply(cloud);
  voxel.apply(cloud);
  EXPECT_EQ(cloud.labels[0], Cloud::kDropped);
  EXPECT_EQ(cloud.labels[1], Cloud::kGround);
  EXPECT_EQ(cloud.labels[2], Cloud::kObstacle);
  EXPECT_EQ(cloud.labels[3], Cloud::kDropped);
  EXPECT_EQ(cloud.labels[4], Cloud::kDropped);
}

TEST(Filters, TheBandsTopFollowsTheCar)
{
  VehicleBox car;
  car.min = Eigen::Vector3f(-1.4f, -0.85f, 0.0f);
  car.max = Eigen::Vector3f(1.45f, 0.85f, 1.85f);
  car.known = true;
  auto scene = [] {
    Cloud cloud;
    cloud.origin = Eigen::Vector3f(0.0f, 0.0f, 1.8f);
    cloud.points = {
      Eigen::Vector3f(10.0f, 0.0f, 1.9f),   // hanging down to 1.9 m: the roof would hit it
      Eigen::Vector3f(10.0f, 1.0f, 2.3f),   // 2.3 m: over the car and its clearance
      Eigen::Vector3f(10.0f, 2.0f, 4.0f),   // a sign
      Eigen::Vector3f(0.5f, 0.2f, 1.7f)};   // the car's own rack
    cloud.labels.assign(cloud.points.size(), Cloud::kObstacle);
    cloud.ground_z.assign(cloud.points.size(), 0.0f);
    return cloud;
  };

  HeightBand band;
  band.setVehicle(car);
  MapParams p;
  p.strings["max_from"] = "vehicle";
  p.doubles["clearance"] = 0.3;  // top: 2.15 m
  band.initialize(p);
  Cloud cloud = scene();
  band.apply(cloud);
  EXPECT_EQ(cloud.labels[0], Cloud::kObstacle);
  EXPECT_EQ(cloud.labels[1], Cloud::kDropped);
  EXPECT_EQ(cloud.labels[2], Cloud::kDropped);

  HeightBand by_sensor;  // top: 1.8 + 0.6 m
  MapParams s;
  s.strings["max_from"] = "sensor";
  s.doubles["clearance"] = 0.6;
  by_sensor.initialize(s);
  cloud = scene();
  by_sensor.apply(cloud);
  EXPECT_EQ(cloud.labels[1], Cloud::kObstacle);
  EXPECT_EQ(cloud.labels[2], Cloud::kDropped);

  CropBox crop;
  crop.setVehicle(car);
  MapParams c;
  c.strings["from"] = "vehicle";
  crop.initialize(c);
  cloud = scene();
  crop.apply(cloud);
  EXPECT_EQ(cloud.labels[0], Cloud::kObstacle);
  EXPECT_EQ(cloud.labels[3], Cloud::kDropped);

  HeightBand unknown;  // no car's box: a clear error, not a silent default
  EXPECT_THROW(unknown.initialize(p), std::invalid_argument);
}

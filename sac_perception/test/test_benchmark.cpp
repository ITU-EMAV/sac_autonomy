// How long the roof lidar's filter chain takes on a full VLP-16 scan, without the simulation
// competing for the CPU. Prints the times; fails only if a scan takes very long.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "sac_perception/filters.hpp"
#include "sac_perception/ground_patchwork.hpp"
#include "sac_perception/grid.hpp"

using namespace sac_perception;

namespace
{
/// A full VLP-16 scan from the roof (16 channels over +-15 deg, 1800 azimuths): a road
/// rising 5 % from 10 m ahead, boxes, and walls around at 35 m
Cloud fullScan()
{
  const Eigen::Vector3f origin(-0.45f, 0.0f, 1.79f);
  struct Box { Eigen::Vector3f min, max; };
  const std::vector<Box> boxes = {
    {{14.5f, 0.5f, 0.0f}, {15.5f, 1.5f, 1.0f}}, {{25.0f, -3.0f, 0.0f}, {29.2f, -1.2f, 1.5f}},
    {{-12.0f, 2.0f, 0.0f}, {-11.0f, 3.0f, 1.0f}}, {{5.0f, -6.0f, 0.0f}, {9.0f, -5.5f, 2.0f}}};
  auto ground = [](float x, float) { return x > 10.0f ? 0.05f * (x - 10.0f) : 0.0f; };
  auto hit = [&](const Eigen::Vector3f & p) {
    if (p.z() <= ground(p.x(), p.y()) || std::hypot(p.x(), p.y()) > 35.0f) {
      return true;
    }
    for (const Box & b : boxes) {
      if ((p.array() >= b.min.array()).all() && (p.array() <= b.max.array()).all()) {
        return true;
      }
    }
    return false;
  };
  std::mt19937 random(3);
  std::normal_distribution<float> noise(0.0f, 0.015f);
  std::vector<Eigen::Vector3f> points;
  for (int channel = -15; channel <= 15; channel += 2) {
    const float elevation = channel * static_cast<float>(M_PI) / 180.0f;
    for (int a = 0; a < 1800; ++a) {
      const float azimuth = a * 0.2f * static_cast<float>(M_PI) / 180.0f;
      const Eigen::Vector3f direction(
        std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth), std::sin(elevation));
      for (float r = 0.5f; r < 100.0f; r += 0.05f) {
        if (hit(origin + r * direction)) {
          points.push_back(origin + (r + noise(random)) * direction);
          break;
        }
      }
    }
  }
  Cloud cloud;
  cloud.resize(points.size());
  cloud.points = points;
  cloud.origin = origin;
  return cloud;
}

double timeFilter(PointFilter & filter, const Cloud & scan, int repeats)
{
  double total = 0.0;
  for (int i = 0; i < repeats; ++i) {
    Cloud cloud = scan;
    const auto start = std::chrono::steady_clock::now();
    filter.apply(cloud);
    total += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  }
  return total / repeats;
}
}  // namespace

TEST(Benchmark, RoofLidarChain)
{
  const Cloud scan = fullScan();
  MapParams none;
  MapParams patchwork_params;
  patchwork_params.doubles["num_min_pts"] = 3;
  MapParams voxel_params;
  voxel_params.doubles["size"] = 0.1;

  CropBox crop;
  crop.initialize(none);
  Voxel voxel;
  voxel.initialize(voxel_params);
  GroundPatchwork patchwork;
  patchwork.initialize(patchwork_params);
  HeightBand band;
  band.initialize(none);

  const int repeats = 20;
  const double t_crop = timeFilter(crop, scan, repeats);
  const double t_voxel = timeFilter(voxel, scan, repeats);
  const double t_patchwork = timeFilter(patchwork, scan, repeats);
  Cloud voxelled = scan;
  voxel.apply(voxelled);
  std::size_t kept = 0;
  for (auto l : voxelled.labels) {
    kept += l != Cloud::kDropped;
  }
  const double t_patchwork_voxelled = timeFilter(patchwork, voxelled, repeats);
  const double t_band = timeFilter(band, scan, repeats);
  std::printf(
    "BENCHMARK %zu points (%zu after a 0.1 m voxel): crop_box %.2f ms, voxel %.2f ms, "
    "ground_patchwork %.2f ms (%.2f ms after the voxel), height_band %.2f ms\n",
    scan.size(), kept, t_crop, t_voxel, t_patchwork, t_patchwork_voxelled, t_band);
  EXPECT_LT(t_patchwork, 200.0);

  // Into the grid: obstacle points mark, ground points clear (as PointCloudSource does)
  Cloud filtered = scan;
  crop.apply(filtered);
  patchwork.apply(filtered);
  band.apply(filtered);
  Scan rays;
  rays.origin = filtered.origin;
  rays.clear_height = 0.3f;
  for (std::size_t k = 0; k < filtered.size(); ++k) {
    if (filtered.labels[k] == Cloud::kDropped) {
      continue;
    }
    Ray ray;
    ray.end = filtered.points[k];
    ray.hit = filtered.labels[k] == Cloud::kObstacle;
    ray.ground_z = std::isnan(filtered.ground_z[k]) ? 0.0f : filtered.ground_z[k];
    rays.rays.push_back(ray);
  }
  RollingGrid grid(80.0, 0.2);
  const int layer = grid.addLayer("roof", LayerParams{});
  grid.recenter(0.0, 0.0);
  double t_integrate = 0.0;
  for (int i = 0; i < repeats; ++i) {
    const auto start = std::chrono::steady_clock::now();
    grid.integrate(layer, rays);
    t_integrate += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  }
  double t_combine = 0.0;
  for (int i = 0; i < repeats; ++i) {
    const auto start = std::chrono::steady_clock::now();
    const auto data = grid.combined();
    t_combine += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    EXPECT_EQ(data.size(), 400u * 400u);
  }
  std::printf(
    "BENCHMARK grid: integrate %zu rays %.2f ms, combine %.2f ms\n", rays.rays.size(), t_integrate / repeats,
    t_combine / repeats);
}

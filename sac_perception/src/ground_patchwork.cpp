#include "sac_perception/ground_patchwork.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

#include <Eigen/Eigenvalues>

namespace sac_perception
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

void meanStdev(const std::deque<double> & values, double & mean, double & stdev)
{
  mean = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  double sum = 0.0;
  for (double v : values) {
    sum += (v - mean) * (v - mean);
  }
  stdev = values.size() > 1 ? std::sqrt(sum / static_cast<double>(values.size() - 1)) : 0.0;
}
}  // namespace

void GroundPatchwork::initialize(const Params & params)
{
  num_iter_ = std::max(1, static_cast<int>(params.getDouble("num_iter", num_iter_)));
  num_lpr_ = std::max(1, static_cast<int>(params.getDouble("num_lpr", num_lpr_)));
  num_min_pts_ = std::max(3, static_cast<int>(params.getDouble("num_min_pts", num_min_pts_)));
  num_rings_of_interest_ = static_cast<int>(params.getDouble("num_rings_of_interest", num_rings_of_interest_));
  th_seeds_ = params.getDouble("th_seeds", th_seeds_);
  th_dist_ = params.getDouble("th_dist", th_dist_);
  th_seeds_v_ = params.getDouble("th_seeds_v", th_seeds_v_);
  th_dist_v_ = params.getDouble("th_dist_v", th_dist_v_);
  min_range_ = params.getDouble("min_range", min_range_);
  max_range_ = params.getDouble("max_range", max_range_);
  uprightness_thr_ = params.getDouble("uprightness_thr", uprightness_thr_);
  adaptive_seed_selection_margin_ = params.getDouble("adaptive_seed_selection_margin", adaptive_seed_selection_margin_);
  max_elevation_storage_ = static_cast<std::size_t>(params.getDouble("max_elevation_storage", 1000));
  max_flatness_storage_ = static_cast<std::size_t>(params.getDouble("max_flatness_storage", 1000));
  enable_rvpf_ = params.getBool("enable_RVPF", enable_rvpf_);
  enable_tgr_ = params.getBool("enable_TGR", enable_tgr_);
  enable_rnr_ = params.getBool("enable_RNR", enable_rnr_);
  rnr_ver_angle_thr_ = params.getDouble("RNR_ver_angle_thr", rnr_ver_angle_thr_);
  rnr_intensity_thr_ = params.getDouble("RNR_intensity_thr", rnr_intensity_thr_);
  sensor_height_ = params.getDouble("sensor_height", -1.0);
  auto readInts = [&](const std::string & key, std::vector<int> & out) {
    if (params.has(key)) {
      out.clear();
      for (double v : params.getDoubles(key, {})) {
        out.push_back(std::max(1, static_cast<int>(v)));
      }
    }
    if (out.size() != 4) {
      throw std::invalid_argument(key + ": 4 values, one per zone");
    }
  };
  readInts("num_sectors_each_zone", num_sectors_each_zone_);
  readInts("num_rings_each_zone", num_rings_each_zone_);
  if (!(min_range_ > 0.0 && max_range_ > min_range_)) {
    throw std::invalid_argument("0 < min_range < max_range");
  }

  // The zones' inner edges: min, then 1/8, 1/4 and 1/2 of the way to max_range
  min_ranges_ = {
    min_range_, (7.0 * min_range_ + max_range_) / 8.0, (3.0 * min_range_ + max_range_) / 4.0,
    (min_range_ + max_range_) / 2.0};
  for (int z = 0; z < 4; ++z) {
    const double outer = z < 3 ? min_ranges_[z + 1] : max_range_;
    ring_sizes_[z] = (outer - min_ranges_[z]) / num_rings_each_zone_[z];
    sector_sizes_[z] = kTwoPi / num_sectors_each_zone_[z];
  }
  const int rings = num_rings_each_zone_[0] + num_rings_each_zone_[1] + num_rings_each_zone_[2] + num_rings_each_zone_[3];
  num_rings_of_interest_ = std::clamp(num_rings_of_interest_, 0, rings);
  enable_agle_ = params.getBool("enable_AGLE", enable_agle_);
  elevation_thr_ = params.getDoubles("elevation_thr", std::vector<double>(num_rings_of_interest_, 0.0));
  flatness_thr_ = params.getDoubles("flatness_thr", std::vector<double>(num_rings_of_interest_, 0.0));
  if (static_cast<int>(elevation_thr_.size()) != num_rings_of_interest_ ||
    static_cast<int>(flatness_thr_.size()) != num_rings_of_interest_)
  {
    throw std::invalid_argument("elevation_thr, flatness_thr: one value per ring of interest");
  }
  update_elevation_.assign(num_rings_of_interest_, {});
  update_flatness_.assign(num_rings_of_interest_, {});

  // The bins, once
  for (int z = 0; z < 4; ++z) {
    czm_[z].assign(num_rings_each_zone_[z], std::vector<std::vector<int>>(num_sectors_each_zone_[z]));
    planes_[z].assign(num_rings_each_zone_[z], std::vector<Plane>(num_sectors_each_zone_[z]));
    is_ground_[z].assign(num_rings_each_zone_[z], std::vector<char>(num_sectors_each_zone_[z], 0));
    ground_points_[z].assign(num_rings_each_zone_[z], std::vector<std::vector<int>>(num_sectors_each_zone_[z]));
  }
}

bool GroundPatchwork::estimatePlane(
  const std::vector<Eigen::Vector3d> & points, const std::vector<int> & indices, Plane & plane) const
{
  if (indices.size() < 3) {
    return false;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int k : indices) {
    mean += points[k];
  }
  mean /= static_cast<double>(indices.size());
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (int k : indices) {
    const Eigen::Vector3d d = points[k] - mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(indices.size());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() != Eigen::Success) {
    return false;
  }
  Eigen::Vector3d normal = solver.eigenvectors().col(0);  // smallest eigenvalue
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  plane.normal = normal;
  plane.mean = mean;
  plane.singular_values = solver.eigenvalues().reverse();
  plane.d = -normal.dot(mean);
  return true;
}

std::vector<int> GroundPatchwork::extractSeeds(
  const std::vector<Eigen::Vector3d> & points, const std::vector<int> & indices, int zone, double threshold) const
{
  // Near the sensor, skip what lies far below the ground (noise) before taking the lowest
  const double floor = zone == 0 ? adaptive_seed_selection_margin_ * sensor_height_ : -std::numeric_limits<double>::infinity();
  heights_.clear();
  for (int k : indices) {
    if (points[k].z() >= floor) {
      heights_.push_back(points[k].z());
    }
  }
  std::vector<int> seeds;
  if (heights_.empty()) {
    return seeds;
  }
  // The lowest num_lpr heights (LPR): a partial selection, no full sort
  const std::size_t count = std::min(heights_.size(), static_cast<std::size_t>(num_lpr_));
  std::nth_element(heights_.begin(), heights_.begin() + (count - 1), heights_.end());
  const double lpr = std::accumulate(heights_.begin(), heights_.begin() + count, 0.0) / static_cast<double>(count);
  for (int k : indices) {
    if (points[k].z() >= floor && points[k].z() < lpr + threshold) {
      seeds.push_back(k);
    }
  }
  return seeds;
}

void GroundPatchwork::apply(Cloud & cloud)
{
  if (sensor_height_ <= 0.0) {
    sensor_height_ = cloud.origin.z();  // over base_footprint: the ground under the car
  }
  statistics_ = Statistics{};
  const std::size_t n = cloud.size();
  const Eigen::Vector3d origin = cloud.origin.cast<double>();
  // Around the sensor: the ground is at about -sensor_height
  std::vector<Eigen::Vector3d> & q = q_;  // kept between scans, as the bins below: no allocation
  q.resize(n);
  for (std::size_t k = 0; k < n; ++k) {
    q[k] = cloud.points[k].cast<double>() - origin;
  }

  // 1. RNR: reflections under the ground seen by the lowest beams, with a weak return
  if (enable_rnr_ && cloud.intensity.size() == n) {
    for (std::size_t k = 0; k < n; ++k) {
      if (cloud.labels[k] == Cloud::kDropped) {
        continue;
      }
      const double r = std::hypot(q[k].x(), q[k].y());
      const double angle = std::atan2(q[k].z(), r) * 180.0 / M_PI;
      if (angle < rnr_ver_angle_thr_ && q[k].z() < -sensor_height_ - 0.8 &&
        cloud.intensity[k] < rnr_intensity_thr_)
      {
        cloud.labels[k] = Cloud::kDropped;
      }
    }
  }

  // 2. CZM: zone, ring and sector of each point (the bins keep their memory between scans)
  auto & czm = czm_;
  auto & planes = planes_;
  auto & is_ground = is_ground_;
  auto & ground_points = ground_points_;
  for (int z = 0; z < 4; ++z) {
    for (int ring = 0; ring < num_rings_each_zone_[z]; ++ring) {
      for (int s = 0; s < num_sectors_each_zone_[z]; ++s) {
        czm[z][ring][s].clear();
        ground_points[z][ring][s].clear();
        is_ground[z][ring][s] = 0;
      }
    }
  }
  std::vector<int> & near_sensor = near_sensor_;  // closer than min_range: kept as they are
  near_sensor.clear();
  for (std::size_t k = 0; k < n; ++k) {
    if (cloud.labels[k] == Cloud::kDropped) {
      continue;
    }
    const double r = std::hypot(q[k].x(), q[k].y());
    if (r > max_range_) {
      cloud.labels[k] = Cloud::kDropped;
      continue;
    }
    if (r <= min_range_) {
      near_sensor.push_back(static_cast<int>(k));
      continue;
    }
    double theta = std::atan2(q[k].y(), q[k].x());
    if (theta < 0.0) {
      theta += kTwoPi;
    }
    const int zone = r < min_ranges_[1] ? 0 : r < min_ranges_[2] ? 1 : r < min_ranges_[3] ? 2 : 3;
    const int ring = std::min(static_cast<int>((r - min_ranges_[zone]) / ring_sizes_[zone]), num_rings_each_zone_[zone] - 1);
    const int sector = std::min(static_cast<int>(theta / sector_sizes_[zone]), num_sectors_each_zone_[zone] - 1);
    czm[zone][ring][sector].push_back(static_cast<int>(k));
  }

  // 3.-4. Per bin: R-VPF, R-GPF, GLE
  struct Candidate
  {
    int zone, ring, sector, concentric;
    double flatness, line;
  };
  std::vector<Candidate> candidates;
  std::vector<std::vector<double>> ringwise_flatness(num_rings_of_interest_);
  std::size_t total_ground = 0;
  std::vector<int> & src = src_;
  std::vector<int> & ground = ground_;
  int concentric = 0;
  for (int z = 0; z < 4; ++z) {
    for (int ring = 0; ring < num_rings_each_zone_[z]; ++ring, ++concentric) {
      for (int s = 0; s < num_sectors_each_zone_[z]; ++s) {
        const std::vector<int> & bin = czm[z][ring][s];
        if (static_cast<int>(bin.size()) < num_min_pts_) {
          statistics_.sparse += bin.size();
          continue;  // too few points: not ground
        }
        // R-VPF: remove vertical structures before looking for the ground
        src = bin;
        if (z == 0 && enable_rvpf_) {
          for (int it = 0; it < num_iter_; ++it) {
            Plane vertical;
            if (!estimatePlane(q, extractSeeds(q, src, z, th_seeds_v_), vertical) ||
              vertical.normal.z() >= uprightness_thr_)
            {
              break;
            }
            std::vector<int> rest;
            rest.reserve(src.size());
            for (int k : src) {
              if (std::abs(vertical.normal.dot(q[k]) + vertical.d) >= th_dist_v_) {
                rest.push_back(k);  // not on the vertical plane (still sorted)
              }
            }
            if (rest.size() == src.size()) {
              break;
            }
            src.swap(rest);
          }
        }

        // R-GPF: the ground plane from the lowest points, refined
        Plane plane;
        bool ok = estimatePlane(q, extractSeeds(q, src, z, th_seeds_), plane);
        for (int it = 0; ok && it < num_iter_; ++it) {
          ground.clear();
          for (int k : src) {
            if (plane.normal.dot(q[k]) + plane.d < th_dist_) {
              ground.push_back(k);
            }
          }
          ok = estimatePlane(q, ground, plane);
        }
        if (!ok) {
          statistics_.no_plane += bin.size();
          continue;
        }
        planes[z][ring][s] = plane;

        // GLE
        const bool is_upright = plane.normal.z() > uprightness_thr_;
        const bool is_near_zone = concentric < num_rings_of_interest_;
        const bool is_heading_outside = plane.mean.dot(plane.normal) < 0.0;  // faces the sensor from below
        const double elevation = plane.mean.z();
        const double flatness = plane.singular_values.minCoeff();
        const double line = plane.singular_values(1) != 0.0 ?
          plane.singular_values(0) / plane.singular_values(1) : std::numeric_limits<double>::infinity();
        const bool is_not_elevated = is_near_zone && elevation < elevation_thr_[concentric];
        const bool is_flat = is_near_zone && flatness < flatness_thr_[concentric];
        if (is_upright && is_not_elevated && is_near_zone) {
          update_elevation_[concentric].push_back(elevation);
          update_flatness_[concentric].push_back(flatness);
        }
        bool accepted = false;
        if (!is_upright) {
          statistics_.not_upright += bin.size();
          accepted = false;
        } else if (!is_near_zone) {
          accepted = true;
        } else if (!is_heading_outside) {
          statistics_.heading += bin.size();
          accepted = false;
        } else if (is_not_elevated || is_flat) {
          accepted = true;
        } else {
          candidates.push_back({z, ring, s, concentric, flatness, line});
        }
        ground_points[z][ring][s] = ground;
        if (accepted) {
          is_ground[z][ring][s] = 1;
          total_ground += ground.size();
          statistics_.off_plane += bin.size() - ground.size();
          if (is_near_zone) {
            ringwise_flatness[concentric].push_back(flatness);
          }
        }
      }
    }
  }

  // 5. TGR: candidates as flat as this scan's ground in their ring, and not a line
  if (enable_tgr_) {
    for (const Candidate & c : candidates) {
      const std::vector<double> & f = ringwise_flatness[c.concentric];
      if (f.size() < 2) {
        statistics_.elevated += czm[c.zone][c.ring][c.sector].size();
        continue;
      }
      std::deque<double> values(f.begin(), f.end());
      double mean = 0.0;
      double stdev = 0.0;
      meanStdev(values, mean, stdev);
      const double limit = mean + 1.5 * stdev;
      double prob_flatness = 1.0 / (1.0 + std::exp((c.flatness - limit) / limit / 10.0));
      if (total_ground > 1500 && c.flatness < th_dist_ * th_dist_) {
        prob_flatness = 1.0;
      }
      const double prob_line = c.line > 8.0 ? 0.0 : 1.0;
      if (prob_line * prob_flatness > 0.5) {
        is_ground[c.zone][c.ring][c.sector] = 1;
        statistics_.reverted += ground_points[c.zone][c.ring][c.sector].size();
        statistics_.off_plane += czm[c.zone][c.ring][c.sector].size() - ground_points[c.zone][c.ring][c.sector].size();
      } else {
        statistics_.elevated += czm[c.zone][c.ring][c.sector].size();
      }
    }
  }

  // Labels, and the ground under every point: its bin's plane, else the nearest ground bin
  // towards the sensor on the same bearing, else the sensor's height below it
  auto groundHeight = [&](const Plane * plane, double x, double y) {
    if (plane == nullptr || std::abs(plane->normal.z()) < 1e-6) {
      return -sensor_height_;
    }
    return -(plane->normal.x() * x + plane->normal.y() * y + plane->d) / plane->normal.z();
  };
  auto nearestGround = [&](int zone, int ring, double theta) -> const Plane * {
    for (int z = zone; z >= 0; --z) {
      const int s = std::min(static_cast<int>(theta / sector_sizes_[z]), num_sectors_each_zone_[z] - 1);
      for (int r = (z == zone ? ring : num_rings_each_zone_[z] - 1); r >= 0; --r) {
        if (is_ground[z][r][s]) {
          return &planes[z][r][s];
        }
      }
    }
    return nullptr;
  };
  for (int z = 0; z < 4; ++z) {
    for (int ring = 0; ring < num_rings_each_zone_[z]; ++ring) {
      for (int s = 0; s < num_sectors_each_zone_[z]; ++s) {
        const std::vector<int> & bin = czm[z][ring][s];
        if (bin.empty()) {
          continue;
        }
        if (is_ground[z][ring][s]) {
          for (int k : ground_points[z][ring][s]) {
            cloud.labels[k] = Cloud::kGround;
          }
        }
        const double theta = (s + 0.5) * sector_sizes_[z];
        const Plane * plane = nearestGround(z, ring, theta);
        // A bin that is not ground (an object in it) often still holds the road beside the
        // object: its lowest points' mean (LPR) is a better ground height than a plane from
        // nearer the sensor, unless the two disagree (no road seen: an object's bottom)
        double lpr = std::numeric_limits<double>::quiet_NaN();
        if (!is_ground[z][ring][s]) {
          std::vector<double> heights;
          heights.reserve(bin.size());
          for (int k : bin) {
            heights.push_back(q[k].z());
          }
          const std::size_t lowest = std::min<std::size_t>(heights.size(), static_cast<std::size_t>(num_lpr_));
          std::partial_sort(heights.begin(), heights.begin() + lowest, heights.end());
          lpr = std::accumulate(heights.begin(), heights.begin() + lowest, 0.0) / static_cast<double>(lowest);
        }
        for (int k : bin) {
          double height = groundHeight(plane, q[k].x(), q[k].y());
          if (std::isfinite(lpr) && std::abs(lpr - height) < 0.5) {
            height = lpr;
          }
          cloud.ground_z[k] = static_cast<float>(origin.z() + height);
        }
      }
    }
  }
  for (int k : near_sensor) {
    cloud.ground_z[k] = static_cast<float>(origin.z() - sensor_height_);
  }

  // 6. A-GLE
  if (enable_agle_) {
    updateThresholds();
  } else {
    for (int i = 0; i < num_rings_of_interest_; ++i) {
      update_elevation_[i].clear();
      update_flatness_[i].clear();
    }
  }
}

void GroundPatchwork::updateThresholds()
{
  for (int i = 0; i < num_rings_of_interest_; ++i) {
    auto & elevations = update_elevation_[i];
    while (elevations.size() > max_elevation_storage_) {
      elevations.pop_front();
    }
    if (!elevations.empty()) {
      double mean = 0.0;
      double stdev = 0.0;
      meanStdev(elevations, mean, stdev);
      if (i == 0) {
        elevation_thr_[i] = mean + 3.0 * stdev;
        sensor_height_ = -mean;
      } else {
        elevation_thr_[i] = mean + 2.0 * stdev;
      }
    }
    auto & flatnesses = update_flatness_[i];
    while (flatnesses.size() > max_flatness_storage_) {
      flatnesses.pop_front();
    }
    if (!flatnesses.empty()) {
      double mean = 0.0;
      double stdev = 0.0;
      meanStdev(flatnesses, mean, stdev);
      flatness_thr_[i] = mean + stdev;
    }
  }
}

}  // namespace sac_perception

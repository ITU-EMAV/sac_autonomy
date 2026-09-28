// Ground segmentation after Patchwork++ (Lee, Lim, Myung, "Patchwork++: Fast and Robust
// Ground Segmentation Solving Partial Under-Segmentation Using 3D Point Cloud", IROS 2022;
// Lim et al., "Patchwork", RA-L 2021), written for this package: the same steps and default
// values, on our Cloud, one instance (and its learnt thresholds) per sensor.
//
// Per scan, around the sensor:
//  1. RNR (reflected noise removal): points of the lowest beams far below the ground with a
//     weak return are reflections, not ground (only when the cloud has intensity).
//  2. CZM (concentric zone model): 4 zones between min_range and max_range, each split into
//     rings and sectors; bins are small near the sensor and large far away.
//  3. Per bin with enough points, sorted by height:
//     R-VPF (zone 0): planes that are not upright (walls, a car's side) are fitted and their
//       points removed before the ground is looked for;
//     R-GPF: seeds are the points within th_seeds of the lowest points' mean (LPR); a plane
//       (PCA) is fitted to them and refined num_iter times with the points within th_dist.
//  4. GLE (ground likelihood estimation) of the bin's plane: upright (normal z >
//     uprightness_thr), and in the rings of interest (near the sensor, where the ground is
//     densely seen) also not elevated (plane height < elevation_thr) or flat (smallest
//     eigenvalue < flatness_thr), and facing the sensor from below.
//  5. TGR (temporal ground revert): bins of the rings of interest that failed only on
//     elevation/flatness become ground again if they are as flat as this scan's ground there
//     and not a line.
//  6. A-GLE: elevation_thr and flatness_thr follow the ground seen so far (mean + k stdev),
//     and so does the sensor's height over the ground.
//
// Additions for the grid: every point gets the height of the ground under it: its bin's
// plane; in a bin that is not ground, the mean of its lowest points (the road beside an
// object) if it agrees within 0.5 m with the nearest ground bin's plane on the way to the
// sensor, else that plane; else the sensor's height below it. The sensor's height starts
// from the cloud's origin (TF), not a parameter.
//
// For a 16-beam lidar (VLP-16) set num_min_pts to 3: its rings are sparse, slopes cut them
// into short pieces per bin, and bins with fewer points are not ground (the default 10 is
// for 64 beams; a 16-beam simulation lap: see DESIGN.md).
//
// Parameters (Patchwork++'s names and defaults):
//   num_iter 3, num_lpr 20, num_min_pts 10, num_rings_of_interest 4, th_seeds 0.125,
//   th_dist 0.125, th_seeds_v 0.25, th_dist_v 0.1, min_range 2.7, max_range 80,
//   uprightness_thr 0.707, adaptive_seed_selection_margin -1.2,
//   num_sectors_each_zone [16, 32, 54, 32], num_rings_each_zone [2, 4, 4, 4],
//   max_elevation_storage 1000, max_flatness_storage 1000, enable_RVPF true, enable_TGR true,
//   enable_RNR true, RNR_ver_angle_thr -15 [deg], RNR_intensity_thr 0.2
// and enable_AGLE (true; not in Patchwork++, where it is always on): off, the thresholds stay
// at their initial elevation_thr / flatness_thr (and sensor_height, if given).
//   elevation_thr, flatness_thr: initial thresholds per ring of interest (Patchwork++: 0)

#pragma once

#include <array>
#include <deque>
#include <vector>

#include <Eigen/Core>

#include "sac_perception/filters.hpp"

namespace sac_perception
{

class GroundPatchwork : public PointFilter
{
public:
  void initialize(const Params & params) override;
  void apply(Cloud & cloud) override;

  /// Why the last scan's points were not ground, by the bin they were in (for tuning)
  struct Statistics
  {
    std::size_t sparse = 0;        // bin with fewer than num_min_pts
    std::size_t no_plane = 0;      // no plane could be fitted
    std::size_t not_upright = 0;
    std::size_t heading = 0;       // plane above the sensor
    std::size_t elevated = 0;      // elevated and not flat, not reverted by TGR
    std::size_t off_plane = 0;     // in a ground bin but off its plane
    std::size_t reverted = 0;      // points of bins TGR made ground
  };
  const Statistics & statistics() const { return statistics_; }

  /// The thresholds learnt so far (A-GLE), per ring of interest
  const std::vector<double> & elevationThresholds() const { return elevation_thr_; }
  const std::vector<double> & flatnessThresholds() const { return flatness_thr_; }
  double sensorHeight() const { return sensor_height_; }

private:
  struct Plane
  {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d singular_values = Eigen::Vector3d::Zero();  // descending
    double d = 0.0;                                             // normal . p + d: height over it
  };
  struct BinResult
  {
    int ring = 0;           // concentric index (all zones' rings in a row)
    Plane plane;
    std::vector<int> ground;
    bool is_ground = false;
  };

  bool estimatePlane(const std::vector<Eigen::Vector3d> & points, const std::vector<int> & indices, Plane & plane) const;
  /// Seeds of a bin: the points below the mean of its num_lpr lowest (LPR) + threshold
  std::vector<int> extractSeeds(
    const std::vector<Eigen::Vector3d> & points, const std::vector<int> & indices, int zone, double threshold) const;
  void updateThresholds();

  // Parameters
  int num_iter_ = 3;
  int num_lpr_ = 20;
  int num_min_pts_ = 10;
  int num_rings_of_interest_ = 4;
  double th_seeds_ = 0.125;
  double th_dist_ = 0.125;
  double th_seeds_v_ = 0.25;
  double th_dist_v_ = 0.1;
  double min_range_ = 2.7;
  double max_range_ = 80.0;
  double uprightness_thr_ = 0.707;
  double adaptive_seed_selection_margin_ = -1.2;
  std::vector<int> num_sectors_each_zone_{16, 32, 54, 32};
  std::vector<int> num_rings_each_zone_{2, 4, 4, 4};
  std::size_t max_elevation_storage_ = 1000;
  std::size_t max_flatness_storage_ = 1000;
  bool enable_rvpf_ = true;
  bool enable_tgr_ = true;
  bool enable_rnr_ = true;
  double rnr_ver_angle_thr_ = -15.0;
  double rnr_intensity_thr_ = 0.2;

  // Zones
  std::array<double, 4> min_ranges_{};
  std::array<double, 4> ring_sizes_{};
  std::array<double, 4> sector_sizes_{};

  // Learnt (A-GLE)
  double sensor_height_ = -1.0;  // < 0: from the first cloud's origin
  std::vector<double> elevation_thr_;
  std::vector<double> flatness_thr_;
  std::vector<std::deque<double>> update_elevation_;
  std::vector<std::deque<double>> update_flatness_;
  bool enable_agle_ = true;
  Statistics statistics_;

  // Kept between scans so that a scan allocates nothing: the bins and scratch space
  std::array<std::vector<std::vector<std::vector<int>>>, 4> czm_;
  std::array<std::vector<std::vector<Plane>>, 4> planes_;
  std::array<std::vector<std::vector<char>>, 4> is_ground_;
  std::array<std::vector<std::vector<std::vector<int>>>, 4> ground_points_;
  std::vector<Eigen::Vector3d> q_;
  std::vector<int> near_sensor_;
  std::vector<int> src_;
  std::vector<int> ground_;
  mutable std::vector<double> heights_;
};

}  // namespace sac_perception

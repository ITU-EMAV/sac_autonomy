// Objects: the obstacle cells seen in the last moment, grouped into clusters, followed over
// time with a velocity each (for what moves: a pedestrian crossing). Both steps are plugins
// (pluginlib), chosen in the YAML:
//
//   objects:
//     window: 0.15            # [s] the cells seen this recently (a 10 Hz lidar's last scan)
//     clusterer: {type: connected_components, ...}
//     tracker: {type: kalman_tracker, ...}
//
// They work in the grid frame (odom), on the ground plane. No ROS here.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "sac_perception/map_representation.hpp"
#include "sac_perception/params.hpp"

namespace sac_perception
{

/// What a cluster looks like by its size
enum class ObjectClass : uint8_t { kUnknown = 0, kPedestrian = 1, kVehicle = 2, kSmall = 3, kStructure = 4 };

struct Cluster
{
  Eigen::Vector2f centre{0.0f, 0.0f};  // its box's
  float yaw = 0.0f;                    // its box's long side
  float length = 0.0f;
  float width = 0.0f;
  float height = 0.0f;                 // its top over the ground (NaN: unknown)
  int points = 0;
  double time = 0.0;                   // [s] its newest point's
  ObjectClass classification = ObjectClass::kUnknown;
  float fresh = 0.0f;                  // the share of its points on cells seen free just before
};

class Clusterer
{
public:
  virtual ~Clusterer() = default;
  virtual void initialize(const Params & params) = 0;
  virtual void cluster(const std::vector<RecentPoint> & points, std::vector<Cluster> & out) = 0;
};

/// Points closer than `tolerance` [m] (0.4) to each other (through others) are one cluster;
/// clusters of fewer than `min_points` (2) are dropped. `fresh`: the share of its points
/// whose cells were seen free within `free_window` [s] (1.0) before they were taken. Each gets the box of its points along
/// their main direction (PCA), and a class by its size:
///   structure   longer than `max_object_size` [m] (8): a wall, a slope, a hedge
///   small       lower than 0.8 m
///   pedestrian  up to 1.2 m across, 1-2.2 m high
///   vehicle     2.5-6 m long
class ConnectedComponents : public Clusterer
{
public:
  void initialize(const Params & params) override;
  void cluster(const std::vector<RecentPoint> & points, std::vector<Cluster> & out) override;

private:
  float tolerance_ = 0.4f;
  int min_points_ = 2;
  float max_object_size_ = 8.0f;
  float free_window_ = 1.0f;
  std::vector<int> parent_;
};

struct Track
{
  uint32_t id = 0;
  Eigen::Vector4f x = Eigen::Vector4f::Zero();          // x, y, vx, vy
  Eigen::Matrix4f P = Eigen::Matrix4f::Identity();
  Cluster last;          // the cluster it was last updated with
  double time = 0.0;     // [s] of its state
  double first = 0.0;    // [s] first seen
  double updated = 0.0;  // [s] last updated
  double fresh_since = -1.0;  // [s] the first of its updates in a row taking cells seen free
  double fresh_last = -1.0;   // [s] the last of them
  uint32_t hits = 0;
  bool confirmed = false;
  bool moving = false;
};

class Tracker
{
public:
  virtual ~Tracker() = default;
  virtual void initialize(const Params & params) = 0;
  /// The clusters of the moment `time`: tracks follow them, new ones start, old ones go
  virtual void update(const std::vector<Cluster> & clusters, double time) = 0;
  virtual const std::vector<Track> & tracks() const = 0;
};

/// A constant-velocity Kalman filter per object (x, y, vx, vy), driven by white noise
/// acceleration (`acceleration_noise` [m/s^2], 2.0); the cluster's centre is the measurement
/// (`position_noise` [m], 0.15, more for a larger cluster: its centre shifts as more of it
/// shows). A cluster joins the track it is nearest to in Mahalanobis distance within the gate
/// (`gate`, 3.0 sigma, and `max_jump` [m], 2.0), greedily, nearest pairs first; one joining
/// no track starts one. A track is `confirmed` after `confirm_hits` (3) scans and dropped
/// `max_unseen` [s] (0.5) after it was last seen (a tentative one after `tentative_unseen`,
/// 0.2). It is `moving` once confirmed, with its speed over `moving_speed` [m/s] (0.5) and
/// the speed's 2-sigma bound over half of it, and, after Wang et al.'s free-space test
/// (DATMO, IJRR 2007), when it has kept taking cells seen free just before: clusters with at
/// least `min_fresh` (0.1) of fresh points, update after update for `fresh_time` [s] (0.3).
/// A long static thing seen in part (a barrier cut by one lidar ring, sliding along it as
/// the car drives) has a moving centre but takes no cell that was free. A cluster much
/// longer than wide is trusted across, hardly along (its ends are where the view ends). A cluster is used once: a grid published at
/// 20 Hz holds a 10 Hz lidar's scan twice, a track takes a cluster only if it is newer than
/// its last. Structures are followed for their place, with no speed.
class KalmanTracker : public Tracker
{
public:
  void initialize(const Params & params) override;
  void update(const std::vector<Cluster> & clusters, double time) override;
  const std::vector<Track> & tracks() const override { return tracks_; }

private:
  void predict(Track & track, double time) const;
  void correct(Track & track, const Cluster & cluster) const;
  Eigen::Matrix2f measurementNoise(const Cluster & cluster) const;

  float acceleration_noise_ = 2.0f;
  float position_noise_ = 0.15f;
  float gate_ = 3.0f;
  float max_jump_ = 2.0f;
  uint32_t confirm_hits_ = 3;
  float max_unseen_ = 0.5f;
  float tentative_unseen_ = 0.2f;
  float moving_speed_ = 0.5f;
  float initial_speed_ = 3.0f;  // [m/s] 1-sigma of a new track's speed
  float min_fresh_ = 0.1f;
  float fresh_time_ = 0.3f;
  std::vector<Track> tracks_;
  uint32_t next_id_ = 1;
};

}  // namespace sac_perception

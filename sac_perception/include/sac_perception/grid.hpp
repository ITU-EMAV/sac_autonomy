// The local occupancy grid: a square window in the odom frame that follows the car (it
// shifts by whole cells and never rotates, so what it has gathered stays where it was seen).
// Each sensor source writes its own layer of log-odds; the layers are combined into one
// grid (a cell is occupied when any layer says so). No ROS in here.
//
// Why odom: fixed to base_footprint, the grid would turn with the car and everything in it
// would have to be moved every step; map jumps when GNSS corrects the global pose, and a
// scan would no longer line up with the previous one. odom is smooth over seconds.

#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace sac_perception
{

/// How a layer turns observations into occupancy (log-odds, per observation).
struct LayerParams
{
  float hit = 0.85f;          // probability of an occupied reading
  float miss = 0.4f;          // probability of a free reading
  float min = -2.0f;          // log-odds clamps: how quickly a cell can change its mind
  float max = 3.5f;
  float decay = 0.0f;         // [1/s] log-odds fade towards unknown (0: keep)
};

/// A ray of a scan, in the grid frame.
struct Ray
{
  Eigen::Vector3f end;
  /// Height of the ground under `end` (the ray is only a free observation where it runs
  /// lower than `clear_height` above it: a 3D lidar ray passes over low obstacles far from
  /// its end). -inf for rays that are free all along (2D lidar).
  float ground_z = -std::numeric_limits<float>::infinity();
  bool hit = true;            // end is an obstacle (false: free space up to and at end)
  bool mark = true;           // a hit that is not marked still leaves its end cell alone
};

struct Scan
{
  Eigen::Vector3f origin{0.0f, 0.0f, 0.0f};  // sensor, in the grid frame
  std::vector<Ray> rays;
  float clear_height = std::numeric_limits<float>::infinity();
  float max_clear_range = 40.0f;             // [m] no clearing beyond (cost)
};

class RollingGrid
{
public:
  /// `size` [m] square, `resolution` [m/cell].
  RollingGrid(double size, double resolution);

  int addLayer(const std::string & name, const LayerParams & params);
  int layerCount() const { return static_cast<int>(layers_.size()); }
  const std::string & layerName(int layer) const { return layers_[layer].name; }

  int width() const { return width_; }
  double resolution() const { return resolution_; }
  /// Grid frame coordinates of the corner of cell (0, 0)
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

  /// Moves the window so that (x, y) is near its centre (whole cells; the content stays put
  /// in the grid frame, what falls out is forgotten). It moves once (x, y) is more than
  /// `recenter_distance` [m] off the centre (moving copies the layers).
  void recenter(double x, double y);
  void setRecenterDistance(double distance) { recenter_distance_ = distance; }

  /// Cell of a point; false outside the window.
  bool cell(double x, double y, int & i, int & j) const;
  int index(int i, int j) const { return j * width_ + i; }

  /// Adds a scan to a layer: hits raise their cells, free rays lower theirs. A cell hit in
  /// this scan is not lowered by another of its rays.
  void integrate(int layer, const Scan & scan);
  /// Sets a cell of a layer (e.g. from another node's grid): log-odds.
  void set(int layer, double x, double y, float log_odds);
  /// Fades all layers by their decay over dt [s].
  void decay(double dt);

  float logOdds(int layer, int i, int j) const { return layers_[layer].cells[index(i, j)]; }

  /// The shared ground height map: sources that find the ground (a 3D lidar's ground points)
  /// write it, sources that cannot tell the ground from an obstacle (a 2D lidar, whose plane
  /// meets a road rising ahead) read it. Heights in the grid frame, with when they were seen.
  void setGround(double x, double y, float z, double time);
  /// The ground height of the nearest cell within `radius` [m] of (x, y) seen within
  /// `max_age` [s] before `time`; false if there is none
  bool groundNear(double x, double y, double radius, double time, double max_age, float & z) const;

  /// All layers combined, as nav_msgs/OccupancyGrid data (row-major from the origin corner):
  /// -1 unknown, 0..100 occupancy probability of the most occupied layer.
  std::vector<int8_t> combined() const;
  /// Occupied cells (combined probability >= threshold).
  std::vector<uint8_t> occupied(float probability_threshold = 0.65f) const;

private:
  struct Layer
  {
    std::string name;
    LayerParams params;
    float hit, miss;  // log-odds steps
    std::vector<float> cells;
  };

  int width_;
  double resolution_;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  bool placed_ = false;
  double recenter_distance_ = 0.0;
  std::vector<Layer> layers_;
  std::vector<uint32_t> hit_scan_;  // per cell: last scan that hit it
  uint32_t scan_counter_ = 0;
  std::vector<float> ground_z_;      // NaN: never seen
  std::vector<double> ground_time_;  // [s] when
};

/// Euclidean distance [m] from each cell to the nearest occupied cell (exact, Felzenszwalb &
/// Huttenlocher's two-pass squared distance transform; linear in the cells). Far from any
/// obstacle (none in the grid): infinity.
std::vector<float> distanceTransform(const std::vector<uint8_t> & occupied, int width, double resolution);

}  // namespace sac_perception

// The filter state: named blocks on a manifold.
//
// Every estimator works on the same State, so estimators, motion models and measurement
// models can be exchanged freely. A state is made of named blocks:
//   - the vehicle's core blocks, always present (see `blocks`), and
//   - blocks that plugins add, e.g. "middle_imu/gyro_bias" or "wheels/scale".
// A block is either a vector or a 3D rotation. Rotations are stored as quaternions but have
// 3 degrees of freedom, so the covariance lives in the tangent space: its size is the sum of
// the blocks' tangent sizes. `boxplus` / `boxminus` move between the two, which is all an
// estimator needs (EKF, error-state EKF and UKF alike).
//
// No ROS in here: the core compiles and is tested without it.

#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace sac_localization
{

/// Time in nanoseconds (the same clock as ROS time, without ROS).
using Stamp = std::int64_t;

inline double toSeconds(Stamp nanoseconds) { return static_cast<double>(nanoseconds) * 1e-9; }
inline Stamp fromSeconds(double seconds) { return static_cast<Stamp>(std::llround(seconds * 1e9)); }

/// Names of the core blocks. Frames: `world` is map (global filter) or odom (local filter),
/// `body` is base_footprint.
namespace blocks
{
inline constexpr char kPosition[] = "position";                     // world [m]
inline constexpr char kOrientation[] = "orientation";               // body in world (rotation)
inline constexpr char kLinearVelocity[] = "linear_velocity";        // body [m/s]
inline constexpr char kAngularVelocity[] = "angular_velocity";      // body [rad/s]
inline constexpr char kLinearAcceleration[] = "linear_acceleration";  // body, without gravity [m/s^2]
}  // namespace blocks

enum class BlockKind
{
  kVector,    // n values
  kRotation,  // a 3D rotation, 3 degrees of freedom
};

struct BlockInfo
{
  std::string name;
  BlockKind kind;
  int tangent_size;    // rows/columns in the covariance
  int tangent_offset;  // first row/column in the covariance
  int storage_index;   // offset in the value vector, or index of the quaternion
};

/// Which blocks a state has and where they are. Fixed once the filter starts.
class StateLayout
{
public:
  bool has(const std::string & block) const;
  /// Throws std::out_of_range for an unknown block.
  const BlockInfo & block(const std::string & name) const;
  const std::vector<BlockInfo> & blocks() const { return blocks_; }
  /// Dimension of the covariance.
  int tangentSize() const { return tangent_size_; }
  int vectorStorageSize() const { return vector_storage_size_; }
  int rotationCount() const { return rotation_count_; }

private:
  friend class StateLayoutBuilder;
  std::vector<BlockInfo> blocks_;
  std::map<std::string, std::size_t> index_;
  int tangent_size_ = 0;
  int vector_storage_size_ = 0;
  int rotation_count_ = 0;
};

/// Collects the blocks before the filter starts: the core blocks, then whatever the motion
/// model and the sensor adapters add.
class StateLayoutBuilder
{
public:
  /// Starts with the core blocks.
  StateLayoutBuilder();
  /// Adds a block. Adding a name that exists with the same shape does nothing (two sensors
  /// may share a block); a different shape throws std::invalid_argument.
  void add(const std::string & name, BlockKind kind, int size = 3);
  std::shared_ptr<const StateLayout> build() const;

private:
  struct Entry
  {
    std::string name;
    BlockKind kind;
    int size;
  };
  std::vector<Entry> entries_;
};

/// A point on the state manifold.
class State
{
public:
  /// All vectors zero, all rotations identity.
  explicit State(std::shared_ptr<const StateLayout> layout);

  const StateLayout & layout() const { return *layout_; }
  const std::shared_ptr<const StateLayout> & layoutPtr() const { return layout_; }

  /// Values of a vector block.
  Eigen::Ref<Eigen::VectorXd> vector(const std::string & block);
  Eigen::Ref<const Eigen::VectorXd> vector(const std::string & block) const;
  /// A rotation block.
  Eigen::Quaterniond & rotation(const std::string & block);
  const Eigen::Quaterniond & rotation(const std::string & block) const;

  // Core blocks
  Eigen::Vector3d position() const { return vector(blocks::kPosition); }
  const Eigen::Quaterniond & orientation() const { return rotation(blocks::kOrientation); }
  Eigen::Vector3d linearVelocity() const { return vector(blocks::kLinearVelocity); }
  Eigen::Vector3d angularVelocity() const { return vector(blocks::kAngularVelocity); }
  Eigen::Vector3d linearAcceleration() const { return vector(blocks::kLinearAcceleration); }

  /// x ⊞ δ: vectors add, rotations turn by δ in their own (body) frame: q * Exp(δ).
  State boxplus(const Eigen::VectorXd & delta) const;
  /// x ⊟ y: the tangent difference, such that y ⊞ (x ⊟ y) == x.
  Eigen::VectorXd boxminus(const State & other) const;

private:
  std::shared_ptr<const StateLayout> layout_;
  Eigen::VectorXd values_;
  std::vector<Eigen::Quaterniond> rotations_;
};

/// What the filter believes at a time: the state and its (tangent) covariance.
struct Belief
{
  Stamp stamp = 0;
  State state;
  Eigen::MatrixXd covariance;
};

}  // namespace sac_localization

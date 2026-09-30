#include "sac_perception/filters.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <unordered_set>

#include <Eigen/Eigenvalues>

namespace sac_perception
{

namespace
{
Eigen::Vector3f readVector3(const Params & params, const std::string & key, const Eigen::Vector3f & fallback)
{
  const std::vector<double> v = params.getDoubles(key, {fallback.x(), fallback.y(), fallback.z()});
  if (v.size() != 3) {
    throw std::invalid_argument(key + " needs 3 values");
  }
  return Eigen::Vector3f(static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]));
}

float getFloat(const Params & params, const std::string & key, float fallback)
{
  return static_cast<float>(params.getDouble(key, fallback));
}
}  // namespace

// ---------------------------------------------------------------- crop box
void CropBox::initialize(const Params & params)
{
  const std::string from = params.getString("from", "box");
  margin_ = getFloat(params, "margin", margin_);
  if (from == "vehicle") {
    if (!vehicle_.known) {
      throw std::invalid_argument("crop_box from: vehicle needs the car's box (vehicle.from in the node)");
    }
    min_ = vehicle_.min - Eigen::Vector3f::Constant(margin_);
    max_ = vehicle_.max + Eigen::Vector3f::Constant(margin_);
  } else if (from == "box") {
    min_ = readVector3(params, "min", min_);
    max_ = readVector3(params, "max", max_);
  } else {
    throw std::invalid_argument("crop_box from: box or vehicle, not '" + from + "'");
  }
  keep_inside_ = params.getBool("keep_inside", keep_inside_);
  max_range_ = getFloat(params, "max_range", max_range_);
}

void CropBox::apply(Cloud & cloud)
{
  for (std::size_t k = 0; k < cloud.size(); ++k) {
    const Eigen::Vector3f & p = cloud.points[k];
    const bool inside = (p.array() >= min_.array()).all() && (p.array() <= max_.array()).all();
    const bool far = std::hypot(p.x() - cloud.origin.x(), p.y() - cloud.origin.y()) > max_range_;
    if (inside != keep_inside_ || far) {
      cloud.labels[k] = Cloud::kDropped;
    }
  }
}

// ---------------------------------------------------------------- ground by height
void GroundHeight::initialize(const Params & params)
{
  height_ = getFloat(params, "height", height_);
}

void GroundHeight::apply(Cloud & cloud)
{
  for (std::size_t k = 0; k < cloud.size(); ++k) {
    if (cloud.labels[k] == Cloud::kDropped) {
      continue;
    }
    cloud.ground_z[k] = 0.0f;
    if (cloud.points[k].z() < height_) {
      cloud.labels[k] = Cloud::kGround;
    }
  }
}

// ---------------------------------------------------------------- height band
void HeightBand::initialize(const Params & params)
{
  min_ = getFloat(params, "min", min_);
  max_ = getFloat(params, "max", max_);
  clearance_ = getFloat(params, "clearance", clearance_);
  const std::string top = params.getString("max_from", "fixed");
  if (top == "fixed") {
    top_ = Top::kFixed;
  } else if (top == "vehicle") {
    if (!vehicle_.known) {
      throw std::invalid_argument("height_band max_from: vehicle needs the car's box (vehicle.from in the node)");
    }
    top_ = Top::kVehicle;
    max_ = vehicle_.height() + clearance_;
  } else if (top == "sensor") {
    top_ = Top::kSensor;
  } else {
    throw std::invalid_argument("height_band max_from: fixed, vehicle or sensor, not '" + top + "'");
  }
}

void HeightBand::apply(Cloud & cloud)
{
  const float top = top_ == Top::kSensor ? cloud.origin.z() + clearance_ : max_;
  for (std::size_t k = 0; k < cloud.size(); ++k) {
    if (cloud.labels[k] != Cloud::kObstacle) {
      continue;
    }
    const float ground = std::isnan(cloud.ground_z[k]) ? 0.0f : cloud.ground_z[k];
    const float height = cloud.points[k].z() - ground;
    if (height < min_) {
      cloud.labels[k] = Cloud::kGround;  // too low to matter: drivable
      cloud.ground_z[k] = cloud.points[k].z();
    } else if (height > top) {
      cloud.labels[k] = Cloud::kDropped;  // overhead
    }
  }
}

// ---------------------------------------------------------------- voxel
void Voxel::initialize(const Params & params)
{
  size_ = getFloat(params, "size", size_);
  if (size_ <= 0.0f) {
    throw std::invalid_argument("voxel size must be positive");
  }
}

void Voxel::apply(Cloud & cloud)
{
  std::unordered_set<int64_t> seen;
  seen.reserve(cloud.size());
  const float inv = 1.0f / size_;
  for (std::size_t k = 0; k < cloud.size(); ++k) {
    if (cloud.labels[k] == Cloud::kDropped) {
      continue;
    }
    const Eigen::Vector3f & p = cloud.points[k];
    const int64_t x = static_cast<int64_t>(std::floor(p.x() * inv)) & 0x1FFFFF;
    const int64_t y = static_cast<int64_t>(std::floor(p.y() * inv)) & 0x1FFFFF;
    const int64_t z = static_cast<int64_t>(std::floor(p.z() * inv)) & 0x1FFFFF;
    if (!seen.insert((x << 42) | (y << 21) | z).second) {
      cloud.labels[k] = Cloud::kDropped;
    }
  }
}

}  // namespace sac_perception

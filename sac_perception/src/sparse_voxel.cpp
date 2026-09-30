#include "sac_perception/sparse_voxel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sac_perception
{

namespace
{
float logit(float p) { return std::log(p / (1.0f - p)); }
constexpr float kForget = 0.05f;  // a voxel fading below this log-odds is dropped
}  // namespace

void SparseVoxel::initialize(const Params & params, const GridGeometry & geometry, const VehicleBox & vehicle)
{
  rules_.read(params, vehicle);
  voxel_height_ = static_cast<float>(params.getDouble("voxel_height", voxel_height_));
  memory_ = params.getBool("memory", memory_);
  window_ = static_cast<float>(params.getDouble("window", window_));
  if (voxel_height_ <= 0.0f) {
    throw std::invalid_argument("map voxel_height must be positive");
  }
  plane_ = std::make_unique<RollingGrid>(geometry.size, geometry.resolution);
  plane_->setRecenterDistance(geometry.recenter_distance);
  plane_->setGroundLevels(rules_.level_gap, rules_.ground_max_age);
  columns_.assign(static_cast<std::size_t>(plane_->width()) * plane_->width(), Column{});
}

int SparseVoxel::addSource(const std::string &, const LayerParams & params)
{
  if (sources_.size() >= 255) {
    throw std::invalid_argument("sparse_voxel: at most 255 sources");
  }
  Source source;
  source.params = params;
  source.hit = logit(params.hit);
  source.miss = logit(params.miss);
  sources_.push_back(source);
  return static_cast<int>(sources_.size()) - 1;
}

void SparseVoxel::recenter(double x, double y)
{
  const double ox = plane_->originX();
  const double oy = plane_->originY();
  plane_->recenter(x, y);
  const double res = plane_->resolution();
  const int di = static_cast<int>(std::lround((plane_->originX() - ox) / res));
  const int dj = static_cast<int>(std::lround((plane_->originY() - oy) / res));
  if (di == 0 && dj == 0) {
    return;
  }
  const int w = plane_->width();
  std::vector<Column> moved(columns_.size());
  for (int j = 0; j < w; ++j) {
    const int sj = j + dj;
    if (sj < 0 || sj >= w) {
      continue;
    }
    for (int i = 0; i < w; ++i) {
      const int si = i + di;
      if (si >= 0 && si < w) {
        moved[plane_->index(i, j)] = std::move(columns_[plane_->index(si, sj)]);
      }
    }
  }
  columns_.swap(moved);
  voxel_count_ = 0;
  for (const Column & c : columns_) {
    voxel_count_ += c.voxels.size();
  }
}

void SparseVoxel::decay(double dt)
{
  plane_->decay(dt);
  std::vector<float> factor(sources_.size());
  for (std::size_t s = 0; s < sources_.size(); ++s) {
    factor[s] = std::exp(-sources_[s].params.decay * static_cast<float>(dt));
  }
  for (Column & column : columns_) {
    if (column.voxels.empty()) {
      continue;
    }
    auto & v = column.voxels;
    for (Voxel & voxel : v) {
      voxel.log_odds *= factor[voxel.source];
    }
    const std::size_t before = v.size();
    v.erase(std::remove_if(v.begin(), v.end(), [](const Voxel & x) { return x.log_odds < kForget; }), v.end());
    voxel_count_ -= before - v.size();
  }
}

void SparseVoxel::hit(
  Column & column, const Source & source, int source_index, const Eigen::Vector3f & p, float now)
{
  const int iz = static_cast<int>(std::floor(p.z() / voxel_height_));
  for (Voxel & voxel : column.voxels) {
    if (voxel.iz == iz) {
      if (voxel.hit_scan != scan_counter_) {
        voxel.hit_scan = scan_counter_;
        voxel.log_odds = std::min(source.params.max, voxel.log_odds + source.hit);
        voxel.source = static_cast<uint8_t>(source_index);
      }
      voxel.time = now;
      voxel.lo = std::min(voxel.lo, p.z());
      voxel.hi = std::max(voxel.hi, p.z());
      return;
    }
  }
  Voxel voxel;
  voxel.iz = static_cast<int16_t>(std::clamp(iz, -32768, 32767));
  voxel.source = static_cast<uint8_t>(source_index);
  voxel.log_odds = std::min(source.params.max, source.hit);
  voxel.lo = voxel.hi = p.z();
  voxel.hit_scan = scan_counter_;
  voxel.time = now;
  column.voxels.push_back(voxel);
  ++voxel_count_;
}

void SparseVoxel::insert(int source_index, const Scan & scan)
{
  const Source & source = sources_.at(source_index);
  if (++scan_counter_ == 0) {  // wrapped: forget which scan touched what
    for (Column & c : columns_) {
      for (Voxel & v : c.voxels) {
        v.hit_scan = 0;
      }
    }
    scan_counter_ = 1;
  }
  time_ = std::max(time_, scan.time);
  if (start_time_ < 0.0) {
    start_time_ = scan.time;
  }
  const float now = static_cast<float>(scan.time - start_time_);
  const int w = plane_->width();
  int i = 0;
  int j = 0;

  if (!memory_) {  // this source's older scans go: the map holds what it sees now
    const float oldest = now - window_;
    for (Column & column : columns_) {
      if (column.voxels.empty()) {
        continue;
      }
      auto & v = column.voxels;
      const std::size_t before = v.size();
      v.erase(
        std::remove_if(
          v.begin(), v.end(),
          [&](const Voxel & x) { return x.source == source_index && x.time < oldest - 1e-4f; }),
        v.end());
      voxel_count_ -= before - v.size();
    }
  }

  // Hits first: this scan's rays do not lower what it hit
  for (const Ray & ray : scan.rays) {
    if (ray.hit && ray.mark && plane_->cell(ray.end.x(), ray.end.y(), i, j)) {
      Column & column = columns_[plane_->index(i, j)];
      hit(column, source, source_index, ray.end, now);
      if (std::isfinite(ray.ground_z)) {
        column.estimated_ground = ray.ground_z;
      }
      column.seen = now;
    } else if (!memory_ && plane_->cell(ray.end.x(), ray.end.y(), i, j)) {
      columns_[plane_->index(i, j)].seen = now;  // no rays traced: where they end was seen
    }
  }
  if (!memory_) {
    return;
  }

  // Free space: each ray's 2D cells, and in each the height it runs at there
  const double res = plane_->resolution();
  const float ox = scan.origin.x();
  const float oy = scan.origin.y();
  const float oz = scan.origin.z();
  const float x0 = static_cast<float>((ox - plane_->originX()) / res);
  const float y0 = static_cast<float>((oy - plane_->originY()) / res);
  const float inf = std::numeric_limits<float>::infinity();
  for (const Ray & ray : scan.rays) {
    const float dx = ray.end.x() - ox;
    const float dy = ray.end.y() - oy;
    const float dz = ray.end.z() - oz;
    const float length = std::hypot(dx, dy);
    if (length < 1e-3f) {
      continue;
    }
    // Up to a cell short of the end (its own voxel: an obstacle, or the ground under a
    // kerb), and within max_clear_range
    const float t_max = std::min(1.0f - static_cast<float>(res) / length, scan.max_clear_range / length);
    if (t_max <= 0.0f) {
      continue;
    }
    const float cx = dx / static_cast<float>(res);
    const float cy = dy / static_cast<float>(res);
    int ci = static_cast<int>(std::floor(x0));
    int cj = static_cast<int>(std::floor(y0));
    const int step_i = cx > 0 ? 1 : -1;
    const int step_j = cy > 0 ? 1 : -1;
    const float delta_i = cx != 0.0f ? std::abs(1.0f / cx) : inf;
    const float delta_j = cy != 0.0f ? std::abs(1.0f / cy) : inf;
    float next_i = cx > 0 ? (ci + 1 - x0) / cx : (cx < 0 ? (x0 - ci) / -cx : inf);
    float next_j = cy > 0 ? (cj + 1 - y0) / cy : (cy < 0 ? (y0 - cj) / -cy : inf);
    float t_in = 0.0f;
    for (int guard = 0; guard < 4 * w; ++guard) {
      const float t_out = std::min({next_i, next_j, t_max});
      if (ci >= 0 && cj >= 0 && ci < w && cj < w) {
        Column & column = columns_[plane_->index(ci, cj)];
        column.seen = now;
        if (!column.voxels.empty()) {
          const float za = oz + t_in * dz;
          const float zb = oz + t_out * dz;
          const int lo = static_cast<int>(std::floor(std::min(za, zb) / voxel_height_));
          const int hi = static_cast<int>(std::floor(std::max(za, zb) / voxel_height_));
          auto & v = column.voxels;
          bool emptied = false;
          for (Voxel & voxel : v) {
            if (voxel.iz >= lo && voxel.iz <= hi && voxel.hit_scan != scan_counter_) {
              voxel.log_odds += source.miss;
              emptied |= voxel.log_odds < kForget;
            }
          }
          if (emptied) {
            const std::size_t before = v.size();
            v.erase(
              std::remove_if(v.begin(), v.end(), [](const Voxel & x) { return x.log_odds < kForget; }), v.end());
            voxel_count_ -= before - v.size();
          }
        }
      }
      if (t_out >= t_max) {
        break;
      }
      t_in = t_out;
      if (next_i < next_j) {
        next_i += delta_i;
        ci += step_i;
      } else {
        next_j += delta_j;
        cj += step_j;
      }
    }
  }
}

void SparseVoxel::set(int source_index, double x, double y, float log_odds)
{
  Source & source = sources_.at(source_index);
  if (source.layer < 0) {  // a 2D source: a layer of its own
    source.layer = plane_->addLayer("source " + std::to_string(source_index), source.params);
  }
  plane_->set(source.layer, x, y, log_odds);
}

float SparseVoxel::ground(int k, int i, int j, double time) const
{
  (void)k;
  const double res = plane_->resolution();
  const double x = plane_->originX() + (i + 0.5) * res;
  const double y = plane_->originY() + (j + 0.5) * res;
  float z = 0.0f;
  if (plane_->groundNear(x, y, rules_.ground_search_radius, time, rules_.ground_max_age, z)) {
    return z;
  }
  return columns_[k].estimated_ground;
}

float SparseVoxel::columnGround(double x, double y) const
{
  int i = 0;
  int j = 0;
  if (!plane_->cell(x, y, i, j)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return ground(plane_->index(i, j), i, j, time_);
}

std::vector<int8_t> SparseVoxel::project() const
{
  const int w = plane_->width();
  std::vector<int8_t> out = plane_->combined();  // the 2D sources, -1 elsewhere
  const float now = static_cast<float>(time_ - start_time_);
  const float recent = rules_.ground_max_age;
  std::vector<float> own_ground;
  if (rules_.max_step > 0.0f) {  // each cell's own ground, for the steps
    own_ground.assign(columns_.size(), std::numeric_limits<float>::quiet_NaN());
    for (int j = 0; j < w; ++j) {
      for (int i = 0; i < w; ++i) {
        const double res = plane_->resolution();
        float z = 0.0f;
        if (plane_->groundNear(
            plane_->originX() + (i + 0.5) * res, plane_->originY() + (j + 0.5) * res, 0.0, time_,
            rules_.ground_max_age, z))
        {
          own_ground[plane_->index(i, j)] = z;
        }
      }
    }
  }
  for (int j = 0; j < w; ++j) {
    for (int i = 0; i < w; ++i) {
      const int k = plane_->index(i, j);
      const Column & column = columns_[k];
      float best = -std::numeric_limits<float>::infinity();
      if (!column.voxels.empty()) {
        const float g = ground(k, i, j, time_);
        for (const Voxel & voxel : column.voxels) {
          if (voxel.log_odds > best && rules_.blocks(g, voxel.lo, voxel.hi)) {
            best = voxel.log_odds;
          }
        }
      }
      int8_t value = -1;
      if (best > -std::numeric_limits<float>::infinity()) {
        value = logOddsToPercent(best);
      } else if (column.seen >= 0.0f && now - column.seen <= recent) {
        value = 0;
      }
      if (!own_ground.empty() && !std::isnan(own_ground[k])) {
        const float g = own_ground[k];
        const int ni[4] = {i + 1, i - 1, i, i};
        const int nj[4] = {j, j, j + 1, j - 1};
        for (int n = 0; n < 4; ++n) {
          if (ni[n] >= 0 && nj[n] >= 0 && ni[n] < w && nj[n] < w) {
            const float gn = own_ground[plane_->index(ni[n], nj[n])];
            if (!std::isnan(gn) && std::abs(gn - g) > rules_.max_step) {
              value = 100;
              break;
            }
          }
        }
      }
      out[k] = std::max(out[k], value);
    }
  }
  return out;
}

void SparseVoxel::points(std::vector<MapPoint> & out) const
{
  out.clear();
  out.reserve(voxel_count_);
  const int w = plane_->width();
  const double res = plane_->resolution();
  for (int j = 0; j < w; ++j) {
    for (int i = 0; i < w; ++i) {
      const int k = plane_->index(i, j);
      const Column & column = columns_[k];
      if (column.voxels.empty()) {
        continue;
      }
      const float g = ground(k, i, j, time_);
      const float x = static_cast<float>(plane_->originX() + (i + 0.5) * res);
      const float y = static_cast<float>(plane_->originY() + (j + 0.5) * res);
      for (const Voxel & voxel : column.voxels) {
        out.push_back(
          {Eigen::Vector3f(x, y, (voxel.iz + 0.5f) * voxel_height_),
            static_cast<float>(logOddsToPercent(voxel.log_odds)),
            static_cast<uint8_t>(rules_.blocks(g, voxel.lo, voxel.hi)), voxel.source});
      }
    }
  }
}

std::vector<std::pair<std::string, double>> SparseVoxel::diagnostics() const
{
  std::size_t columns = 0;
  for (const Column & c : columns_) {
    columns += !c.voxels.empty();
  }
  return {
    {"voxels", static_cast<double>(voxel_count_)}, {"columns", static_cast<double>(columns)},
    {"band_top", rules_.top}};
}

}  // namespace sac_perception

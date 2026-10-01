#include "sac_perception/grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sac_perception
{

namespace
{
float logit(float p) { return std::log(p / (1.0f - p)); }
}  // namespace

RollingGrid::RollingGrid(double size, double resolution)
: width_(std::max(1, static_cast<int>(std::lround(size / resolution)))), resolution_(resolution)
{
  if (resolution <= 0.0) {
    throw std::invalid_argument("grid resolution must be positive");
  }
  hit_scan_.assign(static_cast<std::size_t>(width_) * width_, 0);
  free_time_.assign(static_cast<std::size_t>(width_) * width_, -std::numeric_limits<double>::infinity());
  ground_z_.assign(static_cast<std::size_t>(width_) * width_, std::numeric_limits<float>::quiet_NaN());
  ground_time_.assign(static_cast<std::size_t>(width_) * width_, 0.0);
}

void RollingGrid::setGround(double x, double y, float z, double time)
{
  int i = 0;
  int j = 0;
  if (cell(x, y, i, j)) {
    const int k = index(i, j);
    if (!std::isnan(ground_z_[k]) && z > ground_z_[k] + ground_gap_ && time - ground_time_[k] <= ground_hold_) {
      return;  // a level over the one held
    }
    ground_z_[k] = z;
    ground_time_[k] = time;
  }
}

bool RollingGrid::groundNear(double x, double y, double radius, double time, double max_age, float & z) const
{
  int ci = 0;
  int cj = 0;
  cell(x, y, ci, cj);
  const int reach = static_cast<int>(std::ceil(radius / resolution_));
  int best = std::numeric_limits<int>::max();
  auto look = [&](int i, int j, int d2) {
    if (i < 0 || j < 0 || i >= width_ || j >= width_ || d2 > reach * reach || d2 >= best) {
      return;
    }
    const int k = index(i, j);
    if (!std::isnan(ground_z_[k]) && time - ground_time_[k] <= max_age) {
      best = d2;
      z = ground_z_[k];
    }
  };
  // Ring by ring outwards: every cell of ring r is at least r cells away, so once r * r is
  // past the nearest found there is nothing nearer (the same answer as the whole square)
  for (int r = 0; r <= reach && r * r < best; ++r) {
    if (r == 0) {
      look(ci, cj, 0);
      continue;
    }
    for (int d = -r; d <= r; ++d) {
      look(ci + d, cj - r, d * d + r * r);
      look(ci + d, cj + r, d * d + r * r);
    }
    for (int d = -r + 1; d <= r - 1; ++d) {
      look(ci - r, cj + d, d * d + r * r);
      look(ci + r, cj + d, d * d + r * r);
    }
  }
  return best != std::numeric_limits<int>::max();
}

int RollingGrid::addLayer(const std::string & name, const LayerParams & params)
{
  Layer layer;
  layer.name = name;
  layer.params = params;
  layer.hit = logit(params.hit);
  layer.miss = logit(params.miss);
  layer.cells.assign(static_cast<std::size_t>(width_) * width_, 0.0f);
  layers_.push_back(std::move(layer));
  return static_cast<int>(layers_.size()) - 1;
}

void RollingGrid::recenter(double x, double y)
{
  const double half = width_ * resolution_ / 2.0;
  const double new_x = std::floor((x - half) / resolution_) * resolution_;
  const double new_y = std::floor((y - half) / resolution_) * resolution_;
  if (!placed_) {
    origin_x_ = new_x;
    origin_y_ = new_y;
    placed_ = true;
    return;
  }
  const int di = static_cast<int>(std::lround((new_x - origin_x_) / resolution_));
  const int dj = static_cast<int>(std::lround((new_y - origin_y_) / resolution_));
  // Move only once the car is `recenter_distance` off the centre: moving copies every layer
  const int threshold = static_cast<int>(std::lround(recenter_distance_ / resolution_));
  if (std::abs(di) < std::max(1, threshold) && std::abs(dj) < std::max(1, threshold)) {
    return;
  }
  // Move the content by (-di, -dj) cells; what comes in is unknown
  auto shift = [&](auto & cells, auto empty) {
    auto moved = cells;
    std::fill(moved.begin(), moved.end(), empty);
    for (int j = 0; j < width_; ++j) {
      const int sj = j + dj;
      if (sj < 0 || sj >= width_) {
        continue;
      }
      for (int i = 0; i < width_; ++i) {
        const int si = i + di;
        if (si >= 0 && si < width_) {
          moved[index(i, j)] = cells[index(si, sj)];
        }
      }
    }
    cells.swap(moved);
  };
  for (Layer & layer : layers_) {
    shift(layer.cells, 0.0f);
  }
  shift(hit_scan_, uint32_t{0});
  shift(free_time_, -std::numeric_limits<double>::infinity());
  shift(ground_z_, std::numeric_limits<float>::quiet_NaN());
  shift(ground_time_, 0.0);
  origin_x_ += di * resolution_;
  origin_y_ += dj * resolution_;
}

double RollingGrid::lastFree(double x, double y) const
{
  int i = 0;
  int j = 0;
  return cell(x, y, i, j) ? free_time_[index(i, j)] : -std::numeric_limits<double>::infinity();
}

bool RollingGrid::cell(double x, double y, int & i, int & j) const
{
  i = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  j = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return i >= 0 && j >= 0 && i < width_ && j < width_;
}

void RollingGrid::integrate(int layer_index, const Scan & scan)
{
  Layer & layer = layers_.at(layer_index);
  const float lo = layer.params.min;
  const float hi = layer.params.max;
  ++scan_counter_;
  if (scan_counter_ == 0) {  // wrapped: forget the old marks
    std::fill(hit_scan_.begin(), hit_scan_.end(), 0);
    scan_counter_ = 1;
  }

  // Hits first, so that the free rays of this scan do not lower a cell it also hit
  int i = 0;
  int j = 0;
  for (const Ray & ray : scan.rays) {
    if (ray.hit && ray.mark && cell(ray.end.x(), ray.end.y(), i, j)) {
      const int k = index(i, j);
      if (hit_scan_[k] != scan_counter_) {
        hit_scan_[k] = scan_counter_;
        layer.cells[k] = std::min(hi, layer.cells[k] + layer.hit);
      }
    }
  }

  // Free space: along each ray (grid traversal, Amanatides & Woo), where it runs low enough
  const float ox = scan.origin.x();
  const float oy = scan.origin.y();
  const float oz = scan.origin.z();
  const float inv_res = static_cast<float>(1.0 / resolution_);
  for (const Ray & ray : scan.rays) {
    const float dx = ray.end.x() - ox;
    const float dy = ray.end.y() - oy;
    const float length = std::hypot(dx, dy);
    if (length < 1e-3f) {
      continue;
    }
    // The part of the ray that is below clear_height over the ground under its end, and
    // within max_clear_range: t in [t0, t1]
    float t0 = 0.0f;
    const float dz = ray.end.z() - oz;
    if (std::isfinite(ray.ground_z) && std::isfinite(scan.clear_height)) {
      // height over ground along the ray: (oz - ground) + t dz < clear_height
      const float h0 = oz - ray.ground_z;
      if (dz < -1e-6f) {
        t0 = std::clamp((scan.clear_height - h0) / dz, 0.0f, 1.0f);
      } else if (h0 >= scan.clear_height) {
        continue;  // never low enough
      }
    }
    const float t1 = std::min(1.0f, scan.max_clear_range / length);
    if (t0 >= t1) {
      continue;
    }
    float x = (ox + t0 * dx - static_cast<float>(origin_x_)) * inv_res;
    float y = (oy + t0 * dy - static_cast<float>(origin_y_)) * inv_res;
    const float x_end = (ox + t1 * dx - static_cast<float>(origin_x_)) * inv_res;
    const float y_end = (oy + t1 * dy - static_cast<float>(origin_y_)) * inv_res;
    int ci = static_cast<int>(std::floor(x));
    int cj = static_cast<int>(std::floor(y));
    const int ei = static_cast<int>(std::floor(x_end));
    const int ej = static_cast<int>(std::floor(y_end));
    const int step_i = dx > 0 ? 1 : -1;
    const int step_j = dy > 0 ? 1 : -1;
    const float sx = x_end - x;
    const float sy = y_end - y;
    const float inf = std::numeric_limits<float>::infinity();
    const float delta_i = sx != 0.0f ? std::abs(1.0f / sx) : inf;
    const float delta_j = sy != 0.0f ? std::abs(1.0f / sy) : inf;
    float next_i = sx != 0.0f ? ((step_i > 0 ? (ci + 1 - x) : (x - ci)) * delta_i) : inf;
    float next_j = sy != 0.0f ? ((step_j > 0 ? (cj + 1 - y) : (y - cj)) * delta_j) : inf;
    const bool free_end = !ray.hit;
    for (int guard = 0; guard < 4 * width_; ++guard) {
      const bool at_end = ci == ei && cj == ej;
      if (at_end && !free_end && t1 >= 1.0f) {
        break;  // the hit cell itself
      }
      if (ci < 0 || cj < 0 || ci >= width_ || cj >= width_) {
        if (at_end) {
          break;
        }
      } else {
        const int k = index(ci, cj);
        if (hit_scan_[k] != scan_counter_) {
          layer.cells[k] = std::max(lo, layer.cells[k] + layer.miss);
          free_time_[k] = scan.time;
        }
      }
      if (at_end) {
        break;
      }
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

void RollingGrid::set(int layer, double x, double y, float log_odds)
{
  int i = 0;
  int j = 0;
  if (cell(x, y, i, j)) {
    Layer & l = layers_.at(layer);
    l.cells[index(i, j)] = std::clamp(log_odds, l.params.min, l.params.max);
  }
}

void RollingGrid::decay(double dt)
{
  for (Layer & layer : layers_) {
    if (layer.params.decay <= 0.0f) {
      continue;
    }
    const float factor = std::exp(-layer.params.decay * static_cast<float>(dt));
    for (float & c : layer.cells) {
      c *= factor;
    }
  }
}

int8_t logOddsToPercent(float log_odds)
{
  return static_cast<int8_t>(std::lround(100.0f / (1.0f + std::exp(-std::clamp(log_odds, -10.0f, 10.0f)))));
}

std::vector<int8_t> RollingGrid::combined() const
{
  // Log-odds -> percent through a table (no exp per cell): steps of 0.02 over [-10, 10]
  constexpr float kStep = 0.02f;
  constexpr int kHalf = 500;
  static const std::vector<int8_t> table = [] {
    std::vector<int8_t> t(2 * kHalf + 1);
    for (int q = -kHalf; q <= kHalf; ++q) {
      t[q + kHalf] = static_cast<int8_t>(std::lround(100.0f / (1.0f + std::exp(-q * kStep))));
    }
    return t;
  }();
  const std::size_t n = static_cast<std::size_t>(width_) * width_;
  std::vector<int8_t> out(n, -1);
  constexpr float kUnknown = 1e-3f;
  constexpr float kLowest = -std::numeric_limits<float>::infinity();
  // The most occupied known value per cell, layer by layer (contiguous, vectorizes)
  std::vector<float> best(n, kLowest);
  for (const Layer & layer : layers_) {
    const float * cells = layer.cells.data();
    float * b = best.data();
    for (std::size_t k = 0; k < n; ++k) {
      const float l = cells[k];
      b[k] = (l > kUnknown || l < -kUnknown) ? std::max(b[k], l) : b[k];
    }
  }
  for (std::size_t k = 0; k < n; ++k) {
    if (best[k] != kLowest) {
      const int q = static_cast<int>(std::lround(std::clamp(best[k], -10.0f, 10.0f) / kStep));
      out[k] = table[q + kHalf];
    }
  }
  return out;
}

std::vector<uint8_t> RollingGrid::occupied(float probability_threshold) const
{
  const float threshold = logit(probability_threshold);
  const std::size_t n = static_cast<std::size_t>(width_) * width_;
  std::vector<uint8_t> out(n, 0);
  for (const Layer & layer : layers_) {
    for (std::size_t k = 0; k < n; ++k) {
      if (layer.cells[k] >= threshold) {
        out[k] = 1;
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------- distance transform
namespace
{
/// 1D squared distance transform of f (Felzenszwalb & Huttenlocher 2012)
void edt1d(const float * f, float * d, int n, int * v, float * z)
{
  constexpr float kInf = std::numeric_limits<float>::infinity();
  int k = 0;
  // Skip leading infinite samples: the lower envelope starts at the first finite one
  int first = 0;
  while (first < n && !std::isfinite(f[first])) {
    ++first;
  }
  if (first == n) {
    std::fill(d, d + n, kInf);
    return;
  }
  v[0] = first;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = first + 1; q < n; ++q) {
    if (!std::isfinite(f[q])) {
      continue;
    }
    float s = 0.0f;
    while (true) {
      const int p = v[k];
      s = ((f[q] + q * q) - (f[p] + p * p)) / (2.0f * (q - p));
      if (s <= z[k] && k > 0) {
        --k;
        continue;
      }
      break;
    }
    if (s <= z[k]) {  // k == 0 and the new parabola is lower everywhere
      v[0] = q;
      z[0] = -kInf;
      z[1] = kInf;
      continue;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const float dq = static_cast<float>(q - v[k]);
    d[q] = dq * dq + f[v[k]];
  }
}
}  // namespace

std::vector<float> distanceTransform(const std::vector<uint8_t> & occupied, int width, double resolution)
{
  constexpr float kInf = std::numeric_limits<float>::infinity();
  const std::size_t n = static_cast<std::size_t>(width) * width;
  std::vector<float> grid(n);
  for (std::size_t k = 0; k < n; ++k) {
    grid[k] = occupied[k] ? 0.0f : kInf;
  }
  std::vector<float> f(width), d(width), z(width + 1);
  std::vector<int> v(width);
  // Columns, then rows
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < width; ++j) {
      f[j] = grid[static_cast<std::size_t>(j) * width + i];
    }
    edt1d(f.data(), d.data(), width, v.data(), z.data());
    for (int j = 0; j < width; ++j) {
      grid[static_cast<std::size_t>(j) * width + i] = d[j];
    }
  }
  for (int j = 0; j < width; ++j) {
    float * row = grid.data() + static_cast<std::size_t>(j) * width;
    std::copy(row, row + width, f.begin());
    edt1d(f.data(), d.data(), width, v.data(), z.data());
    for (int i = 0; i < width; ++i) {
      row[i] = std::isfinite(d[i]) ? std::sqrt(d[i]) * static_cast<float>(resolution) : kInf;
    }
  }
  return grid;
}

}  // namespace sac_perception

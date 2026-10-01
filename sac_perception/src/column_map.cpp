#include "sac_perception/column_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace sac_perception
{

namespace
{
float logit(float p) { return std::log(p / (1.0f - p)); }
constexpr float kForget = 0.05f;  // an element fading below this log-odds is dropped
}  // namespace

void ColumnMap::initialize(const Params & params, const GridGeometry & geometry, const VehicleBox & vehicle)
{
  rules_.read(params, vehicle);
  memory_ = params.getBool("memory", memory_);
  window_ = static_cast<float>(params.getDouble("window", window_));
  merge_rays_ = params.getBool("merge_rays", merge_rays_);
  ground_cache_time_ = static_cast<float>(params.getDouble("ground_cache_time", ground_cache_time_));
  configure(params);
  plane_ = std::make_unique<RollingGrid>(geometry.size, geometry.resolution);
  plane_->setRecenterDistance(geometry.recenter_distance);
  plane_->setGroundLevels(rules_.level_gap, rules_.ground_max_age);
  columns_.assign(static_cast<std::size_t>(plane_->width()) * plane_->width(), Column{});
  blocks_ = (plane_->width() + kBlock - 1) / kBlock;
  block_full_.assign(static_cast<std::size_t>(blocks_) * blocks_, 0);
  block_seen_.assign(static_cast<std::size_t>(blocks_) * blocks_, -1.0f);
}

void ColumnMap::markBlocks()
{
  std::fill(block_full_.begin(), block_full_.end(), 0);
  const int w = plane_->width();
  for (int j = 0; j < w; ++j) {
    for (int i = 0; i < w; ++i) {
      if (!columns_[plane_->index(i, j)].elements.empty()) {
        block_full_[(j / kBlock) * blocks_ + i / kBlock] = 1;
      }
    }
  }
}

int ColumnMap::addSource(const std::string &, const LayerParams & params)
{
  if (sources_.size() >= 255) {
    throw std::invalid_argument("map: at most 255 sources");
  }
  Source source;
  source.params = params;
  source.hit = logit(params.hit);
  source.miss = logit(params.miss);
  sources_.push_back(source);
  return static_cast<int>(sources_.size()) - 1;
}

void ColumnMap::recenter(double x, double y)
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
  std::fill(block_seen_.begin(), block_seen_.end(), -1.0f);  // the blocks moved: seen again next scan
  markBlocks();
}

void ColumnMap::forget(Column & column)
{
  auto & e = column.elements;
  e.erase(std::remove_if(e.begin(), e.end(), [](const Element & x) { return x.log_odds < kForget; }), e.end());
}

void ColumnMap::decay(double dt)
{
  plane_->decay(dt);
  std::vector<float> factor(sources_.size());
  for (std::size_t s = 0; s < sources_.size(); ++s) {
    factor[s] = std::exp(-sources_[s].params.decay * static_cast<float>(dt));
  }
  forEachFull([&](int, int, int k) {
    Column & column = columns_[k];
    for (Element & element : column.elements) {
      element.log_odds *= factor[element.source];
    }
    forget(column);
  });
}

void ColumnMap::raise(Element & element, const Eigen::Vector3f & p)
{
  if (element.hit_scan != scan_counter_) {
    element.hit_scan = scan_counter_;
    element.log_odds = std::min(source_->params.max, element.log_odds + source_->hit);
    element.source = static_cast<uint8_t>(source_index_);
  }
  element.time = now_;
  element.lo = std::min(element.lo, p.z());
  element.hi = std::max(element.hi, p.z());
}

ColumnMap::Element & ColumnMap::make(Column & column, const Eigen::Vector3f & p)
{
  Element element;
  element.lo = element.hi = p.z();
  element.log_odds = std::min(source_->params.max, source_->hit);
  element.time = now_;
  element.hit_scan = scan_counter_;
  element.iz = 0;
  element.source = static_cast<uint8_t>(source_index_);
  column.elements.push_back(element);
  return column.elements.back();
}

void ColumnMap::insert(int source_index, const Scan & scan)
{
  const Source & source = sources_.at(source_index);
  if (++scan_counter_ == 0) {  // wrapped: forget which scan touched what
    for (Column & c : columns_) {
      for (Element & e : c.elements) {
        e.hit_scan = 0;
      }
    }
    scan_counter_ = 1;
  }
  time_ = std::max(time_, scan.time);
  if (start_time_ < 0.0) {
    start_time_ = scan.time;
  }
  const float now = static_cast<float>(scan.time - start_time_);
  source_ = &source;
  source_index_ = source_index;
  now_ = now;
  const int w = plane_->width();
  int i = 0;
  int j = 0;

  if (!memory_) {  // this source's older scans go: the map holds what it sees now
    const float oldest = now - window_;
    forEachFull([&](int, int, int k) {
      auto & e = columns_[k].elements;
      e.erase(
        std::remove_if(
          e.begin(), e.end(),
          [&](const Element & x) { return x.source == source_index && x.time < oldest - 1e-4f; }),
        e.end());
    });
  }

  // Hits first: this scan's rays do not lower what it hit
  for (const Ray & ray : scan.rays) {
    if (!plane_->cell(ray.end.x(), ray.end.y(), i, j)) {
      continue;
    }
    Column & column = columns_[plane_->index(i, j)];
    if (ray.hit && ray.mark) {
      add(column, ray.end);
      if (std::isfinite(ray.ground_z)) {
        column.estimated_ground = ray.ground_z;
      }
      column.seen = now;
    } else if (!memory_) {
      column.seen = now;  // no rays traced: where they end was seen
    }
  }
  markBlocks();
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
  trace_.clear();
  if (merge_rays_) {  // one ray per end cell and height, to the mean of their ends
    std::unordered_map<uint64_t, uint32_t> slot;
    slot.reserve(scan.rays.size());
    std::vector<uint32_t> count;
    const float inv = static_cast<float>(1.0 / res);
    for (const Ray & ray : scan.rays) {
      const auto ix = static_cast<int64_t>(std::floor((ray.end.x() - plane_->originX()) * inv));
      const auto iy = static_cast<int64_t>(std::floor((ray.end.y() - plane_->originY()) * inv));
      const auto iz = static_cast<int64_t>(std::floor(ray.end.z() / element_height_));
      const uint64_t key = (static_cast<uint64_t>((ix + (1 << 20)) & 0x1FFFFF) << 42) |
        (static_cast<uint64_t>((iy + (1 << 20)) & 0x1FFFFF) << 21) |
        static_cast<uint64_t>((iz + (1 << 20)) & 0x1FFFFF);
      const auto [it, added] = slot.try_emplace(key, static_cast<uint32_t>(trace_.size()));
      if (added) {
        trace_.push_back(ray.end);
        count.push_back(1);
      } else {
        trace_[it->second] += ray.end;
        ++count[it->second];
      }
    }
    for (std::size_t k = 0; k < trace_.size(); ++k) {
      trace_[k] /= static_cast<float>(count[k]);
    }
  } else {
    for (const Ray & ray : scan.rays) {
      trace_.push_back(ray.end);
    }
  }
  for (const Eigen::Vector3f & end : trace_) {
    const float dx = end.x() - ox;
    const float dy = end.y() - oy;
    const float dz = end.z() - oz;
    const float length = std::hypot(dx, dy);
    if (length < 1e-3f) {
      continue;
    }
    // Up to a cell short of the end (its own column: an obstacle, or the ground under a
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
      if (ci >= 0 && cj >= 0 && ci < w && cj < w) {
        const int b = (cj / kBlock) * blocks_ + ci / kBlock;
        block_seen_[b] = now;
        if (!block_full_[b]) {  // nothing to lower in this block: to where the ray leaves it
          const int bi = (ci / kBlock) * kBlock;
          const int bj = (cj / kBlock) * kBlock;
          const float exit_i = cx > 0 ? (bi + kBlock - x0) / cx : (cx < 0 ? (bi - x0) / cx : inf);
          const float exit_j = cy > 0 ? (bj + kBlock - y0) / cy : (cy < 0 ? (bj - y0) / cy : inf);
          const float t_exit = std::min(exit_i, exit_j);
          if (t_exit >= t_max) {
            break;
          }
          // Restart the traversal just past the block's edge (a thousandth of a cell past
          // it); if rounding leaves it in the block, step on cell by cell instead
          const float t = t_exit + 1e-3f / std::max(std::abs(cx), std::abs(cy));
          const int ni = static_cast<int>(std::floor(x0 + t * cx));
          const int nj = static_cast<int>(std::floor(y0 + t * cy));
          if ((ni >= bi && ni < bi + kBlock && nj >= bj && nj < bj + kBlock) || t >= t_max) {
            if (t >= t_max) {
              break;
            }
          } else {
            ci = ni;
            cj = nj;
            next_i = cx > 0 ? (ci + 1 - x0) / cx : (cx < 0 ? (ci - x0) / cx : inf);
            next_j = cy > 0 ? (cj + 1 - y0) / cy : (cy < 0 ? (cj - y0) / cy : inf);
            t_in = t_exit;
            continue;
          }
        }
      }
      const float t_out = std::min({next_i, next_j, t_max});
      if (ci >= 0 && cj >= 0 && ci < w && cj < w) {
        Column & column = columns_[plane_->index(ci, cj)];
        column.seen = now;
        if (!column.elements.empty()) {
          const float za = oz + t_in * dz;
          const float zb = oz + t_out * dz;
          const float z_min = std::min(za, zb);
          const float z_max = std::max(za, zb);
          bool emptied = false;
          for (Element & element : column.elements) {
            if (element.hit_scan != scan_counter_ && crosses(element, z_min, z_max)) {
              element.log_odds += source.miss;
              emptied |= element.log_odds < kForget;
            }
          }
          if (emptied) {
            forget(column);
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

void ColumnMap::set(int source_index, double x, double y, float log_odds)
{
  Source & source = sources_.at(source_index);
  if (source.layer < 0) {  // a 2D source: a layer of its own
    source.layer = plane_->addLayer("source " + std::to_string(source_index), source.params);
  }
  plane_->set(source.layer, x, y, log_odds);
}

float ColumnMap::ground(int k, int i, int j, double time) const
{
  const Column & column = columns_[k];
  const float now = static_cast<float>(time - start_time_);
  if (now >= column.ground_at && now - column.ground_at < ground_cache_time_) {
    return column.ground;
  }
  column.ground = findGround(k, i, j, time);
  column.ground_at = now;
  return column.ground;
}

float ColumnMap::findGround(int k, int i, int j, double time) const
{
  const double res = plane_->resolution();
  const double x = plane_->originX() + (i + 0.5) * res;
  const double y = plane_->originY() + (j + 0.5) * res;
  float z = 0.0f;
  if (plane_->groundNear(x, y, 0.0, time, rules_.ground_max_age, z) ||
    plane_->groundNear(x, y, rules_.ground_search_radius, time, rules_.ground_max_age, z))
  {
    return z;
  }
  return columns_[k].estimated_ground;
}

float ColumnMap::columnGround(double x, double y) const
{
  int i = 0;
  int j = 0;
  if (!plane_->cell(x, y, i, j)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return findGround(plane_->index(i, j), i, j, time_);
}

std::vector<std::pair<float, float>> ColumnMap::column(double x, double y) const
{
  std::vector<std::pair<float, float>> out;
  int i = 0;
  int j = 0;
  if (plane_->cell(x, y, i, j)) {
    for (const Element & e : columns_[plane_->index(i, j)].elements) {
      out.emplace_back(e.lo, e.hi);
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<int8_t> ColumnMap::project() const
{
  const int w = plane_->width();
  std::vector<int8_t> out = plane_->combined();  // the 2D sources, -1 elsewhere
  const float now = static_cast<float>(time_ - start_time_);
  const float recent = rules_.ground_max_age;
  std::vector<float> own_ground;
  if (rules_.max_step > 0.0f) {  // each cell's own ground, for the steps
    own_ground.assign(columns_.size(), std::numeric_limits<float>::quiet_NaN());
    const double res = plane_->resolution();
    for (int j = 0; j < w; ++j) {
      for (int i = 0; i < w; ++i) {
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
      const int b = (j / kBlock) * blocks_ + i / kBlock;
      float best = -std::numeric_limits<float>::infinity();
      if (block_full_[b] && !column.elements.empty()) {
        const float g = ground(k, i, j, time_);
        for (const Element & element : column.elements) {
          if (element.log_odds > best && rules_.blocks(g, element.lo, element.hi)) {
            best = element.log_odds;
          }
        }
      }
      int8_t value = -1;
      if (best > -std::numeric_limits<float>::infinity()) {
        value = logOddsToPercent(best);
      } else if (
        (block_seen_[b] >= 0.0f && now - block_seen_[b] <= recent) ||
        (column.seen >= 0.0f && now - column.seen <= recent))
      {
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

void ColumnMap::points(std::vector<MapPoint> & out) const
{
  out.clear();
  const int w = plane_->width();
  const double res = plane_->resolution();
  std::vector<float> heights;
  (void)w;
  forEachFull([&](int i, int j, int k) {
    const Column & column = columns_[k];
    {
      const float g = ground(k, i, j, time_);
      const float x = static_cast<float>(plane_->originX() + (i + 0.5) * res);
      const float y = static_cast<float>(plane_->originY() + (j + 0.5) * res);
      for (const Element & element : column.elements) {
        const float occupancy = static_cast<float>(logOddsToPercent(element.log_odds));
        const auto blocks = static_cast<uint8_t>(rules_.blocks(g, element.lo, element.hi));
        heights.clear();
        draw(element, heights);
        for (float z : heights) {
          out.push_back({Eigen::Vector3f(x, y, z), occupancy, blocks, element.source});
        }
      }
    }
  });
}

std::vector<std::pair<std::string, double>> ColumnMap::diagnostics() const
{
  std::size_t columns = 0;
  std::size_t elements = 0;
  for (const Column & c : columns_) {
    columns += !c.elements.empty();
    elements += c.elements.size();
  }
  return {
    {elementName(), static_cast<double>(elements)}, {"columns", static_cast<double>(columns)},
    {"band_top", rules_.top}};
}

}  // namespace sac_perception

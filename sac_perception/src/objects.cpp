#include "sac_perception/objects.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>

#include <Eigen/Eigenvalues>

namespace sac_perception
{

namespace
{
/// Convex hull, counter-clockwise (Andrew's monotone chain); a point or two as they are
std::vector<Eigen::Vector2f> convexHull(std::vector<Eigen::Vector2f> p)
{
  std::sort(p.begin(), p.end(), [](const Eigen::Vector2f & a, const Eigen::Vector2f & b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  p.erase(std::unique(p.begin(), p.end()), p.end());
  if (p.size() < 3) {
    return p;
  }
  auto cross = [](const Eigen::Vector2f & o, const Eigen::Vector2f & a, const Eigen::Vector2f & b) {
    return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
  };
  std::vector<Eigen::Vector2f> h(2 * p.size());
  std::size_t k = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0.0f) {
      --k;
    }
    h[k++] = p[i];
  }
  for (std::size_t i = p.size() - 1, t = k + 1; i-- > 0;) {
    while (k >= t && cross(h[k - 2], h[k - 1], p[i]) <= 0.0f) {
      --k;
    }
    h[k++] = p[i];
  }
  h.resize(k - 1);
  return h;
}

/// The smallest circle around the points (Welzl, iteratively)
void enclosingCircle(const std::vector<Eigen::Vector2f> & p, Eigen::Vector2f & centre, float & radius)
{
  auto inside = [&](const Eigen::Vector2f & q) { return (q - centre).norm() <= radius + 1e-4f; };
  auto two = [&](const Eigen::Vector2f & a, const Eigen::Vector2f & b) {
    centre = (a + b) / 2.0f;
    radius = (a - b).norm() / 2.0f;
  };
  auto three = [&](const Eigen::Vector2f & a, const Eigen::Vector2f & b, const Eigen::Vector2f & c) {
    const float d = 2.0f * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()));
    if (std::abs(d) < 1e-9f) {  // in a line: the two furthest apart
      two(a, b);
      if (!inside(c)) {
        two(a, c);
      }
      if (!inside(b)) {
        two(b, c);
      }
      return;
    }
    const float a2 = a.squaredNorm(), b2 = b.squaredNorm(), c2 = c.squaredNorm();
    centre = Eigen::Vector2f(
      (a2 * (b.y() - c.y()) + b2 * (c.y() - a.y()) + c2 * (a.y() - b.y())) / d,
      (a2 * (c.x() - b.x()) + b2 * (a.x() - c.x()) + c2 * (b.x() - a.x())) / d);
    radius = (a - centre).norm();
  };
  centre = p.empty() ? Eigen::Vector2f::Zero() : p[0];
  radius = 0.0f;
  for (std::size_t i = 1; i < p.size(); ++i) {
    if (inside(p[i])) {
      continue;
    }
    centre = p[i];
    radius = 0.0f;
    for (std::size_t j = 0; j < i; ++j) {
      if (inside(p[j])) {
        continue;
      }
      two(p[i], p[j]);
      for (std::size_t k = 0; k < j; ++k) {
        if (!inside(p[k])) {
          three(p[i], p[j], p[k]);
        }
      }
    }
  }
}

/// A rectangle to an L shape: the direction (0-90 degrees) maximizing the closeness of the
/// points to the rectangle's nearer edges (Zhang et al. 2017)
void lShape(const std::vector<Eigen::Vector2f> & p, Eigen::Vector2f & centre, float & yaw, float & length, float & width)
{
  constexpr float kMinDistance = 0.01f;  // d0
  float best = -1.0f;
  float best_theta = 0.0f;
  std::vector<float> c1(p.size()), c2(p.size());
  for (int step = 0; step < 90; ++step) {
    const float theta = static_cast<float>(step) * static_cast<float>(M_PI) / 180.0f;
    const Eigen::Vector2f e1(std::cos(theta), std::sin(theta));
    const Eigen::Vector2f e2(-e1.y(), e1.x());
    float lo1 = std::numeric_limits<float>::infinity(), hi1 = -lo1, lo2 = lo1, hi2 = -lo1;
    for (std::size_t i = 0; i < p.size(); ++i) {
      c1[i] = p[i].dot(e1);
      c2[i] = p[i].dot(e2);
      lo1 = std::min(lo1, c1[i]);
      hi1 = std::max(hi1, c1[i]);
      lo2 = std::min(lo2, c2[i]);
      hi2 = std::max(hi2, c2[i]);
    }
    float closeness = 0.0f;
    for (std::size_t i = 0; i < p.size(); ++i) {
      const float d1 = std::min(hi1 - c1[i], c1[i] - lo1);
      const float d2 = std::min(hi2 - c2[i], c2[i] - lo2);
      closeness += 1.0f / std::max(std::min(d1, d2), kMinDistance);
    }
    if (closeness > best) {
      best = closeness;
      best_theta = theta;
    }
  }
  const Eigen::Vector2f e1(std::cos(best_theta), std::sin(best_theta));
  const Eigen::Vector2f e2(-e1.y(), e1.x());
  float lo1 = std::numeric_limits<float>::infinity(), hi1 = -lo1, lo2 = lo1, hi2 = -lo1;
  for (const auto & q : p) {
    lo1 = std::min(lo1, q.dot(e1));
    hi1 = std::max(hi1, q.dot(e1));
    lo2 = std::min(lo2, q.dot(e2));
    hi2 = std::max(hi2, q.dot(e2));
  }
  centre = e1 * (lo1 + hi1) / 2.0f + e2 * (lo2 + hi2) / 2.0f;
  length = hi1 - lo1;
  width = hi2 - lo2;
  yaw = best_theta;
  if (width > length) {
    std::swap(length, width);
    yaw += static_cast<float>(M_PI) / 2.0f;
  }
}
}  // namespace

// ---------------------------------------------------------------- clustering
void ConnectedComponents::initialize(const Params & params)
{
  tolerance_ = static_cast<float>(params.getDouble("tolerance", tolerance_));
  min_points_ = static_cast<int>(params.getDouble("min_points", min_points_));
  max_object_size_ = static_cast<float>(params.getDouble("max_object_size", max_object_size_));
}

void ConnectedComponents::cluster(const std::vector<RecentPoint> & points, std::vector<Cluster> & out)
{
  out.clear();
  const int n = static_cast<int>(points.size());
  parent_.resize(n);
  std::iota(parent_.begin(), parent_.end(), 0);
  auto find = [&](int a) {
    while (parent_[a] != a) {
      parent_[a] = parent_[parent_[a]];
      a = parent_[a];
    }
    return a;
  };
  // Points into cells of the tolerance's size: neighbours are in the 3 x 3 cells around
  const float inv = 1.0f / tolerance_;
  auto key = [](int64_t i, int64_t j) {
    return (static_cast<uint64_t>(i + (1 << 30)) << 32) | static_cast<uint64_t>(j + (1 << 30));
  };
  std::unordered_map<uint64_t, std::vector<int>> cells;
  cells.reserve(points.size());
  std::vector<std::pair<int64_t, int64_t>> cell_of(n);
  for (int k = 0; k < n; ++k) {
    const int64_t i = static_cast<int64_t>(std::floor(points[k].xy.x() * inv));
    const int64_t j = static_cast<int64_t>(std::floor(points[k].xy.y() * inv));
    cell_of[k] = {i, j};
    cells[key(i, j)].push_back(k);
  }
  const float tolerance2 = tolerance_ * tolerance_;
  for (int k = 0; k < n; ++k) {
    const auto [ci, cj] = cell_of[k];
    for (int64_t dj = -1; dj <= 1; ++dj) {
      for (int64_t di = -1; di <= 1; ++di) {
        const auto it = cells.find(key(ci + di, cj + dj));
        if (it == cells.end()) {
          continue;
        }
        for (int m : it->second) {
          if (m > k && (points[m].xy - points[k].xy).squaredNorm() <= tolerance2) {
            const int a = find(k);
            const int b = find(m);
            if (a != b) {
              parent_[a] = b;
            }
          }
        }
      }
    }
  }
  std::unordered_map<int, std::vector<int>> groups;
  for (int k = 0; k < n; ++k) {
    groups[find(k)].push_back(k);
  }
  for (const auto & [root, members] : groups) {
    (void)root;
    if (static_cast<int>(members.size()) < min_points_) {
      continue;
    }
    Cluster c;
    c.points = static_cast<int>(members.size());
    c.height = std::numeric_limits<float>::quiet_NaN();
    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    int fresh = 0;
    for (int m : members) {
      mean += points[m].xy;
      fresh += points[m].dynamic;
      c.time = std::max(c.time, points[m].time);
      if (std::isfinite(points[m].top)) {
        c.height = std::isnan(c.height) ? points[m].top : std::max(c.height, points[m].top);
      }
    }
    mean /= static_cast<float>(members.size());
    c.fresh = static_cast<float>(fresh) / static_cast<float>(members.size());
    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (int m : members) {
      const Eigen::Vector2f d = points[m].xy - mean;
      cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen(cov);
    const Eigen::Vector2f major = eigen.eigenvectors().col(1);  // the larger eigenvalue's
    const Eigen::Vector2f minor(-major.y(), major.x());
    float lo_a = std::numeric_limits<float>::infinity(), hi_a = -lo_a, lo_b = lo_a, hi_b = -lo_a;
    for (int m : members) {
      const Eigen::Vector2f d = points[m].xy - mean;
      lo_a = std::min(lo_a, d.dot(major));
      hi_a = std::max(hi_a, d.dot(major));
      lo_b = std::min(lo_b, d.dot(minor));
      hi_b = std::max(hi_b, d.dot(minor));
    }
    c.yaw = std::atan2(major.y(), major.x());
    c.length = hi_a - lo_a;
    c.width = hi_b - lo_b;
    c.centre = mean + major * (lo_a + hi_a) / 2.0f + minor * (lo_b + hi_b) / 2.0f;
    if (c.length > max_object_size_) {
      c.classification = ObjectClass::kStructure;
    } else if (std::isnan(c.height)) {
      c.classification = ObjectClass::kUnknown;
    } else if (c.height < 0.8f) {
      c.classification = ObjectClass::kSmall;
    } else if (c.length <= 1.2f && c.width <= 1.2f && c.height >= 1.0f && c.height <= 2.2f) {
      c.classification = ObjectClass::kPedestrian;
    } else if (c.length >= 2.5f && c.length <= 6.0f) {
      c.classification = ObjectClass::kVehicle;
    }
    // Its shape, by its class
    std::vector<Eigen::Vector2f> xy(members.size());
    for (std::size_t k = 0; k < members.size(); ++k) {
      xy[k] = points[members[k]].xy;
    }
    switch (c.classification) {
      case ObjectClass::kVehicle:
        c.shape = ObjectShape::kBox;
        lShape(xy, c.centre, c.yaw, c.length, c.width);
        break;
      case ObjectClass::kPedestrian:
      case ObjectClass::kSmall: {
        c.shape = ObjectShape::kCylinder;
        float radius = 0.0f;
        enclosingCircle(xy, c.centre, radius);
        c.length = c.width = 2.0f * radius;
        c.yaw = 0.0f;
        break;
      }
      case ObjectClass::kStructure: {
        c.shape = ObjectShape::kPolygons;
        std::unordered_map<uint64_t, std::vector<Eigen::Vector2f>> tiles;
        for (const auto & q : xy) {
          tiles[key(static_cast<int64_t>(std::floor(q.x() / 2.0f)), static_cast<int64_t>(std::floor(q.y() / 2.0f)))]
          .push_back(q);
        }
        for (auto & [k, tile] : tiles) {
          (void)k;
          c.footprint.push_back(convexHull(tile));
        }
        break;
      }
      default:
        c.shape = ObjectShape::kPolygons;
        c.footprint.push_back(convexHull(xy));
    }
    out.push_back(c);
  }
}

// ---------------------------------------------------------------- tracking
void KalmanTracker::initialize(const Params & params)
{
  acceleration_noise_ = static_cast<float>(params.getDouble("acceleration_noise", acceleration_noise_));
  position_noise_ = static_cast<float>(params.getDouble("position_noise", position_noise_));
  gate_ = static_cast<float>(params.getDouble("gate", gate_));
  max_jump_ = static_cast<float>(params.getDouble("max_jump", max_jump_));
  confirm_hits_ = static_cast<uint32_t>(params.getDouble("confirm_hits", confirm_hits_));
  max_unseen_ = static_cast<float>(params.getDouble("max_unseen", max_unseen_));
  tentative_unseen_ = static_cast<float>(params.getDouble("tentative_unseen", tentative_unseen_));
  moving_speed_ = static_cast<float>(params.getDouble("moving_speed", moving_speed_));
  initial_speed_ = static_cast<float>(params.getDouble("initial_speed", initial_speed_));
  min_fresh_ = static_cast<float>(params.getDouble("min_fresh", min_fresh_));
  fresh_time_ = static_cast<float>(params.getDouble("fresh_time", fresh_time_));
}

void KalmanTracker::predict(Track & track, double time) const
{
  const float dt = static_cast<float>(time - track.time);
  if (dt <= 0.0f) {
    return;
  }
  Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  const float a2 = acceleration_noise_ * acceleration_noise_;
  Eigen::Matrix4f Q = Eigen::Matrix4f::Zero();
  for (int axis = 0; axis < 2; ++axis) {
    Q(axis, axis) = dt * dt * dt * dt / 4.0f * a2;
    Q(axis, axis + 2) = Q(axis + 2, axis) = dt * dt * dt / 2.0f * a2;
    Q(axis + 2, axis + 2) = dt * dt * a2;
  }
  track.x = F * track.x;
  track.P = F * track.P * F.transpose() + Q;
  track.time = time;
}

Eigen::Matrix2f KalmanTracker::measurementNoise(const Cluster & cluster) const
{
  // A larger cluster's centre shifts as more or less of it shows; a long thin one's ends are
  // where the view of it ends: trusted across, hardly along
  const float sigma = position_noise_ + 0.1f * std::max(cluster.length, cluster.width);
  if (cluster.length < 2.0f * cluster.width + 0.3f) {
    return Eigen::Matrix2f::Identity() * sigma * sigma;
  }
  const float along = sigma + 0.5f * cluster.length;
  const Eigen::Vector2f u(std::cos(cluster.yaw), std::sin(cluster.yaw));
  const Eigen::Vector2f v(-u.y(), u.x());
  return along * along * u * u.transpose() + sigma * sigma * v * v.transpose();
}

void KalmanTracker::correct(Track & track, const Cluster & cluster) const
{
  const Eigen::Matrix2f S = track.P.topLeftCorner<2, 2>() + measurementNoise(cluster);
  const Eigen::Matrix<float, 4, 2> K = track.P.leftCols<2>() * S.inverse();
  track.x += K * (cluster.centre - track.x.head<2>());
  Eigen::Matrix<float, 2, 4> H = Eigen::Matrix<float, 2, 4>::Zero();
  H(0, 0) = H(1, 1) = 1.0f;
  track.P = (Eigen::Matrix4f::Identity() - K * H) * track.P;
  track.P = 0.5f * (track.P + track.P.transpose());
}

void KalmanTracker::update(const std::vector<Cluster> & clusters, double time)
{
  for (Track & track : tracks_) {
    predict(track, time);
  }
  // Candidate pairs within the gate; a cluster no newer than a track's last is that track's
  // own scan again: it pairs (so it starts no track) but does not update it
  struct Pair
  {
    float distance;
    std::size_t track, cluster;
    bool again;
  };
  std::vector<Pair> pairs;
  for (std::size_t t = 0; t < tracks_.size(); ++t) {
    const Track & track = tracks_[t];
    const Eigen::Matrix2f S = track.P.topLeftCorner<2, 2>() + measurementNoise(track.last);
    const Eigen::Matrix2f S_inv = S.inverse();
    for (std::size_t c = 0; c < clusters.size(); ++c) {
      const Eigen::Vector2f d = clusters[c].centre - track.x.head<2>();
      if (d.norm() > max_jump_) {
        continue;
      }
      const float m2 = d.dot(S_inv * d);
      if (m2 > gate_ * gate_) {
        continue;
      }
      pairs.push_back({m2, t, c, clusters[c].time <= track.updated + 1e-6});
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair & a, const Pair & b) { return a.distance < b.distance; });
  std::vector<bool> track_taken(tracks_.size(), false);
  std::vector<bool> cluster_taken(clusters.size(), false);
  for (const Pair & p : pairs) {
    if (track_taken[p.track] || cluster_taken[p.cluster]) {
      continue;
    }
    track_taken[p.track] = true;
    cluster_taken[p.cluster] = true;
    if (p.again) {
      continue;
    }
    Track & track = tracks_[p.track];
    const Cluster & cluster = clusters[p.cluster];
    correct(track, cluster);
    if (cluster.classification == ObjectClass::kStructure) {
      track.x.tail<2>().setZero();  // its centre moves as more of it shows: no speed
    }
    if (cluster.fresh >= min_fresh_) {  // taking cells seen free: in a row?
      if (track.fresh_last < 0.0 || cluster.time - track.fresh_last > 0.25) {
        track.fresh_since = cluster.time;
      }
      track.fresh_last = cluster.time;
    }
    track.last = cluster;
    track.updated = cluster.time;
    ++track.hits;
    track.confirmed = track.confirmed || track.hits >= confirm_hits_;
  }
  // New tracks
  for (std::size_t c = 0; c < clusters.size(); ++c) {
    if (cluster_taken[c]) {
      continue;
    }
    Track track;
    track.id = next_id_++;
    track.x << clusters[c].centre, 0.0f, 0.0f;
    track.P = Eigen::Matrix4f::Zero();
    track.P.topLeftCorner<2, 2>() = measurementNoise(clusters[c]);
    track.P(2, 2) = track.P(3, 3) = initial_speed_ * initial_speed_;
    track.last = clusters[c];
    track.time = time;
    track.first = track.updated = clusters[c].time;
    track.hits = 1;
    track.confirmed = confirm_hits_ <= 1;
    tracks_.push_back(track);
  }
  // Old ones go; what moves
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [&](const Track & t) { return time - t.updated > (t.confirmed ? max_unseen_ : tentative_unseen_); }),
    tracks_.end());
  for (Track & track : tracks_) {
    const Eigen::Vector2f v = track.x.tail<2>();
    const float speed = v.norm();
    if (track.moving && track.confirmed && track.last.classification != ObjectClass::kStructure &&
      speed > moving_speed_ && track.fresh_last >= 0.0 && time - track.fresh_last <= 1.5)
    {
      continue;  // it keeps moving while it went on taking fresh cells lately
    }
    track.moving = false;
    const bool fresh = track.fresh_last >= 0.0 && time - track.fresh_last <= 0.5 &&
      track.fresh_last - track.fresh_since >= fresh_time_ - 1e-6;
    if (track.confirmed && fresh && track.last.classification != ObjectClass::kStructure && speed > moving_speed_) {
      const Eigen::Vector2f u = v / speed;
      const float sigma = std::sqrt(std::max(0.0f, u.dot(track.P.bottomRightCorner<2, 2>() * u)));
      track.moving = speed - 2.0f * sigma > 0.5f * moving_speed_;
    }
  }
}

}  // namespace sac_perception

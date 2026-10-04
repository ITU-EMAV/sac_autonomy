#ifndef PLANNER__LATTICE_GENERATOR_HPP_
#define PLANNER__LATTICE_GENERATOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace Lattice
{

struct Point
{
    double x;
    double y;
};

struct Path
{
    std::vector<Point> points;
    double lateral_offset;  // meters, signed, relative to the centerline
    double cost;
    bool blocked;
};

/**
 * Lattice trajectory generator.
 *
 * Adapted from the RC car's lattice_planner_pkg for the full-size vehicle.
 * Works entirely in the vehicle frame (vehicle at the origin, x forward):
 * picks an anchor point on the local centerline at a speed-adaptive lookahead
 * distance, spawns candidate goal points laterally offset from it, connects
 * each with a cubic Hermite spline and scores the splines against the local
 * occupancy grid.
 *
 * The grid is expected to be vehicle-centred and vehicle-aligned, which is
 * what smart_car::OccupancyGrid publishes (origin at pose - size/2 in both
 * axes, rotated by the vehicle yaw, so the vehicle sits on the centre cell).
 *
 * Scoring matches the A* cost policy: value 0 is the preferred surface, every
 * value below the lethal threshold stays drivable but costs more, and only
 * lethal cells block a candidate outright.
 *
 * Selection policy (in priority order):
 *   1. If the centerline candidate (offset 0) is clear, take it.
 *   2. Otherwise stick with the previously chosen lateral offset if it is
 *      still clear (avoids oscillating between sides).
 *   3. Otherwise pick the cheapest clear candidate (lateral offset + surface
 *      cost, plus a penalty for jumping far from the current offset).
 *   4. If everything is blocked, fall back to the centerline.
 *
 * Not thread-safe; call the setters and computeTrajectory from one thread.
 */
class Generator
{
public:
    Generator() = default;

    /**
     * Point the generator at a local occupancy grid.
     * `grid_data` is borrowed, not copied: the caller must keep the underlying
     * message alive until the next setGridInfo() call.
     */
    void setGridInfo(int width, int height, double resolution,
                     const std::vector<int8_t> &grid_data)
    {
        grid_width_ = width;
        grid_height_ = height;
        resolution_ = resolution;
        grid_data_ = &grid_data;
    }

    /// Centerline in the vehicle frame, ordered along the driving direction.
    void setCenterline(std::vector<Point> centerline)
    {
        centerline_ = std::move(centerline);
    }

    void setSpeedLimits(double min_speed, double max_speed)
    {
        min_speed_ = min_speed;
        max_speed_ = max_speed;
    }

    void setLookaheadDistances(double min_lookahead, double max_lookahead)
    {
        min_lookahead_ = min_lookahead;
        max_lookahead_ = max_lookahead;
    }

    /// Cells at or above this occupancy value block a candidate outright.
    void setLethalThreshold(int threshold) { lethal_threshold_ = std::max(1, threshold); }
    void setPathResolution(double resolution) { path_resolution_ = std::max(1e-3, resolution); }
    void setCandidateOffsets(std::vector<double> offsets)
    {
        candidate_deltas_ = std::move(offsets);
    }

    /// Cost charged per meter of lateral offset from the centerline.
    void setOffsetWeight(double weight) { offset_weight_ = weight; }

    /**
     * Cost of a candidate that runs entirely over lethal-valued surface,
     * expressed in the same units as the lateral offset (meters). A candidate
     * over value-50 cells with a lethal threshold of 100 pays half of it.
     */
    void setOffLanePenalty(double penalty) { off_lane_penalty_ = penalty; }

    /// Forget the committed lateral offset (call when leaving lattice mode).
    void resetSelection() { locked_delta_ = 0.0; }

    /**
     * Generate candidates and return the selected trajectory (vehicle frame).
     * Returns an empty vector when no usable centerline is available ahead.
     */
    std::vector<Point> computeTrajectory(double current_speed)
    {
        last_candidates_.clear();
        if (centerline_.empty())
            return {};

        updateLookahead(current_speed);

        size_t anchor_idx = 0;
        if (!findAnchorIndex(anchor_idx))
            return {};  // the whole window is behind the vehicle

        const double road_yaw = estimateRoadYaw(anchor_idx);
        const Point &anchor = centerline_[anchor_idx];

        // Unit normal of the road direction: lateral offsets run along it.
        const double nx = -std::sin(road_yaw);
        const double ny = std::cos(road_yaw);

        last_candidates_.reserve(candidate_deltas_.size());
        for (const double delta : candidate_deltas_)
        {
            Path path;
            path.lateral_offset = delta;
            generateCubicSpline(anchor.x + nx * delta, anchor.y + ny * delta,
                                road_yaw, path.points);

            double mean_value = 0.0;
            path.blocked = evaluatePath(path.points, mean_value);

            // Prefer staying near the centerline, and prefer value-0 surface.
            path.cost = offset_weight_ * std::abs(delta) +
                        off_lane_penalty_ * (mean_value / lethal_threshold_);

            last_candidates_.push_back(std::move(path));
        }

        return selectBestCandidate();
    }

    /// Candidates from the last computeTrajectory() call, for visualization.
    const std::vector<Path> &getAllTrajectories() const { return last_candidates_; }

    /// Lateral offset currently committed to, in meters.
    double getLockedOffset() const { return locked_delta_; }

private:
    void updateLookahead(double current_speed)
    {
        const double span = max_speed_ - min_speed_;
        if (span <= 1e-6)
        {
            lookahead_distance_ = min_lookahead_;
            return;
        }
        const double clamped = std::clamp(current_speed, min_speed_, max_speed_);
        const double ratio = (clamped - min_speed_) / span;
        lookahead_distance_ = min_lookahead_ + ratio * (max_lookahead_ - min_lookahead_);
    }

    /**
     * Centerline point ahead of the vehicle whose distance is closest to the
     * current lookahead distance. Returns false when nothing lies ahead.
     */
    bool findAnchorIndex(size_t &anchor_idx) const
    {
        double min_diff = std::numeric_limits<double>::max();
        bool found = false;
        for (size_t i = 0; i < centerline_.size(); ++i)
        {
            if (centerline_[i].x <= 0.0)
                continue;
            const double dist = std::hypot(centerline_[i].x, centerline_[i].y);
            const double diff = std::abs(dist - lookahead_distance_);
            if (diff < min_diff)
            {
                min_diff = diff;
                anchor_idx = i;
                found = true;
            }
        }
        return found;
    }

    double estimateRoadYaw(size_t idx) const
    {
        if (idx + 1 < centerline_.size())
        {
            return std::atan2(centerline_[idx + 1].y - centerline_[idx].y,
                              centerline_[idx + 1].x - centerline_[idx].x);
        }
        if (idx > 0)
        {
            return std::atan2(centerline_[idx].y - centerline_[idx - 1].y,
                              centerline_[idx].x - centerline_[idx - 1].x);
        }
        return 0.0;
    }

    std::vector<Point> selectBestCandidate()
    {
        // 1. Strict priority: a clear centerline wins immediately.
        int centerline_idx = -1;
        for (size_t i = 0; i < last_candidates_.size(); ++i)
        {
            if (std::abs(last_candidates_[i].lateral_offset) < 1e-3)
            {
                centerline_idx = static_cast<int>(i);
                break;
            }
        }
        if (centerline_idx != -1 && !last_candidates_[centerline_idx].blocked)
        {
            locked_delta_ = 0.0;
            return last_candidates_[centerline_idx].points;
        }

        // 2. Centerline is blocked: keep the previously locked offset if it is
        //    still clear, for temporal consistency.
        for (const auto &candidate : last_candidates_)
        {
            if (!candidate.blocked &&
                std::abs(candidate.lateral_offset - locked_delta_) < 1e-3)
            {
                return candidate.points;
            }
        }

        // 3. Cheapest clear candidate, biased towards the side already taken.
        int best_idx = -1;
        double best_cost = std::numeric_limits<double>::max();
        for (size_t i = 0; i < last_candidates_.size(); ++i)
        {
            const auto &candidate = last_candidates_[i];
            if (candidate.blocked)
                continue;

            double cost = candidate.cost;
            const bool same_side = (locked_delta_ * candidate.lateral_offset) > 0.0;
            if (same_side && std::abs(locked_delta_) > 0.01)
                cost *= 0.8;
            cost += std::abs(candidate.lateral_offset - locked_delta_);

            if (cost < best_cost)
            {
                best_cost = cost;
                best_idx = static_cast<int>(i);
            }
        }
        if (best_idx != -1)
        {
            locked_delta_ = last_candidates_[best_idx].lateral_offset;
            return last_candidates_[best_idx].points;
        }

        // 4. Everything blocked (often a false positive at this point): fall
        //    back to the centerline rather than stopping dead.
        if (centerline_idx != -1)
            return last_candidates_[centerline_idx].points;

        return {};
    }

    /// Cubic Hermite spline from the vehicle pose (origin, heading +x) to the
    /// goal point, with the road heading as the exit tangent.
    void generateCubicSpline(double gx, double gy, double gyaw,
                             std::vector<Point> &out_points) const
    {
        out_points.clear();
        const double dist = std::hypot(gx, gy);
        const double scale = dist * 1.2;  // tangent magnitude

        const double mx0 = scale;
        const double my0 = 0.0;
        const double mx1 = scale * std::cos(gyaw);
        const double my1 = scale * std::sin(gyaw);

        const int steps = std::max(5, static_cast<int>(dist / path_resolution_));
        out_points.reserve(steps + 1);

        for (int i = 0; i <= steps; ++i)
        {
            const double t = static_cast<double>(i) / steps;
            const double t2 = t * t;
            const double t3 = t2 * t;

            const double h10 = t3 - 2.0 * t2 + t;
            const double h01 = -2.0 * t3 + 3.0 * t2;
            const double h11 = t3 - t2;

            out_points.push_back({h10 * mx0 + h01 * gx + h11 * mx1,
                                  h10 * my0 + h01 * gy + h11 * my1});
        }
    }

    /**
     * Score a candidate against the local grid (vehicle on the centre cell).
     * Returns true when any sample hits a lethal cell, and writes the mean
     * occupancy value over the sampled cells to `mean_value`.
     */
    bool evaluatePath(const std::vector<Point> &points, double &mean_value) const
    {
        mean_value = 0.0;
        if (!grid_data_ || points.empty())
            return false;  // no grid yet: assume clear

        const int center_x = grid_width_ / 2;
        const int center_y = grid_height_ / 2;
        const double inv_res = 1.0 / resolution_;

        double sum = 0.0;
        size_t counted = 0;
        bool blocked = false;

        for (const auto &pt : points)
        {
            // floor, not truncation: samples behind or right of the vehicle
            // have negative coordinates and would otherwise land one cell off.
            const int idx_x = center_x + static_cast<int>(std::floor(pt.x * inv_res));
            const int idx_y = center_y + static_cast<int>(std::floor(pt.y * inv_res));

            if (idx_x < 0 || idx_x >= grid_width_ || idx_y < 0 || idx_y >= grid_height_)
                continue;

            const size_t index = static_cast<size_t>(idx_y) * grid_width_ + idx_x;
            if (index >= grid_data_->size())
                continue;

            const int value = (*grid_data_)[index];
            if (value >= lethal_threshold_)
            {
                blocked = true;
                break;
            }
            // Unknown cells count as lethal-adjacent but stay drivable.
            sum += (value < 0) ? lethal_threshold_ : value;
            ++counted;
        }

        if (counted > 0)
            mean_value = sum / static_cast<double>(counted);
        return blocked;
    }

    // -- Grid (borrowed) ------------------------------------------------------
    int grid_width_ = 0;
    int grid_height_ = 0;
    double resolution_ = 0.3;
    const std::vector<int8_t> *grid_data_ = nullptr;

    // -- Inputs / outputs -----------------------------------------------------
    std::vector<Point> centerline_;
    std::vector<Path> last_candidates_;

    // -- Tuning (defaults sized for the full-size vehicle) --------------------
    double min_lookahead_ = 8.0;
    double max_lookahead_ = 12.0;
    double min_speed_ = 1.0;
    double max_speed_ = 2.0;
    double lookahead_distance_ = 8.0;
    double path_resolution_ = 0.2;  // meters between spline samples
    // Left-first within each magnitude: equal-cost candidates are broken by
    // list order, so a symmetric obstacle is passed on the overtaking side.
    std::vector<double> candidate_deltas_ =
        {0.0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0, 2.5, -2.5, 3.0, -3.0};
    int lethal_threshold_ = 100;
    double offset_weight_ = 1.0;
    double off_lane_penalty_ = 6.0;

    // -- Selection state ------------------------------------------------------
    double locked_delta_ = 0.0;  // lateral offset currently committed to
};

}  // namespace Lattice

#endif  // PLANNER__LATTICE_GENERATOR_HPP_

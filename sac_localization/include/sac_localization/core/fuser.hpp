// Fuser: time handling around an estimator, the same for every engine.
//
// Sensors arrive at different rates and with different delays (a GNSS fix is often 100 ms
// old, 2 m at 20 m/s). The fuser keeps a short history of beliefs, one after each fused
// measurement:
//   - measurements are fused in stamp order;
//   - a measurement (or input) older than the last fused one rewinds to the belief before it
//     and replays the measurements after it (like robot_localization's smooth_lagged_data);
//   - one older than the whole history is dropped and reported as "too late";
//   - predictions are split into steps of at most max_prediction_step;
//   - the belief at `now` is a prediction from the last fused measurement, so new
//     measurements that are newer than the last one never cause a rewind.

#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "sac_localization/core/estimator.hpp"
#include "sac_localization/core/measurement.hpp"
#include "sac_localization/core/motion_model.hpp"
#include "sac_localization/core/state.hpp"

namespace sac_localization
{

struct FuserOptions
{
  double history = 1.0;               // [s] how late a measurement may be
  double max_prediction_step = 0.01;  // [s]
};

class Fuser
{
public:
  Fuser(
    std::shared_ptr<const StateLayout> layout, std::shared_ptr<Estimator> estimator,
    std::shared_ptr<MotionModel> motion_model, FuserOptions options);

  /// Starts (or restarts) the filter from a belief, clearing the history.
  void reset(const Belief & initial);
  /// Stops the filter (e.g. after the clock jumped back); initialized() is false until reset().
  void clear();
  bool initialized() const { return initialized_; }

  /// Adds `variance` to the covariance diagonal of a block (the filter admits it may be lost
  /// there), from the latest fused belief on.
  void widen(const std::string & block, const Eigen::VectorXd & variance);

  /// Queue a measurement or an input; they are used at the next update().
  void add(Measurement measurement);
  void add(Input input);

  /// Fuses everything queued with a stamp up to `now`, then predicts to `now`. Returns what
  /// happened to each new measurement (replayed ones are not reported again). When `now` is
  /// more than a second before the filter's time (the clock jumped back) the filter clears
  /// itself.
  std::vector<UpdateResult> update(Stamp now);

  /// The belief at the time of the last update().
  const Belief & belief() const { return *current_; }
  /// The belief predicted to `stamp` from the last fused measurement, without changing the
  /// filter.
  Belief predicted(Stamp stamp) const;

  const StateLayout & layout() const { return *layout_; }
  std::shared_ptr<const StateLayout> layoutPtr() const { return layout_; }
  MotionModel & motionModel() { return *motion_model_; }
  Estimator & estimator() { return *estimator_; }

private:
  /// Predicts `belief` forward to `stamp` in steps, with the inputs in effect.
  void advance(Belief & belief, Stamp stamp) const;
  Inputs inputsAt(Stamp stamp) const;
  void prune(Stamp now);

  std::shared_ptr<const StateLayout> layout_;
  std::shared_ptr<Estimator> estimator_;
  std::shared_ptr<MotionModel> motion_model_;
  FuserOptions options_;
  bool initialized_ = false;

  std::deque<Belief> history_;           // after each fused measurement, oldest first
  std::deque<Measurement> processed_;    // fused, within the history, in stamp order
  std::vector<Measurement> pending_;     // not fused yet
  std::deque<Input> inputs_;             // within the history, in stamp order
  Stamp earliest_new_input_ = 0;             // oldest input added since the last update
  bool has_new_input_ = false;
  std::optional<Belief> current_;        // the belief at the last update
};

}  // namespace sac_localization

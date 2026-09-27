// Fuser: time handling around an estimator, the same for every engine.
//
// Sensors arrive at different rates and with different delays (a GNSS fix is often 100 ms
// old, 2 m at 20 m/s). The fuser keeps a short history of beliefs, measurements and inputs:
//   - measurements are fused in stamp order;
//   - a measurement older than the current belief rewinds to the belief before it and
//     replays everything after it (like robot_localization's smooth_lagged_data);
//   - one older than the history is dropped and reported ("too late");
//   - predictions are split into steps of at most max_prediction_step.

#pragma once

#include <deque>
#include <memory>
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
    std::shared_ptr<const StateLayout> layout, std::unique_ptr<Estimator> estimator,
    std::unique_ptr<MotionModel> motion_model, FuserOptions options);

  /// Starts (or restarts) the filter from a belief, clearing the history.
  void reset(const Belief & initial);
  bool initialized() const { return initialized_; }

  /// Queue a measurement or an input; they are used at the next update().
  void add(Measurement measurement);
  void add(Input input);

  /// Fuses everything queued with a stamp up to `now`, then predicts to `now`. Returns what
  /// happened to each measurement.
  std::vector<UpdateResult> update(Stamp now);

  /// The current belief (at the time of the last update()).
  const Belief & belief() const;
  /// The belief predicted to `stamp`, without changing the filter (for outputs between
  /// updates).
  Belief predicted(Stamp stamp) const;

  const StateLayout & layout() const { return *layout_; }
  const MotionModel & motionModel() const { return *motion_model_; }

private:
  struct Step
  {
    Belief belief;          // after this step
    Inputs inputs;          // inputs in effect after this step
  };

  std::shared_ptr<const StateLayout> layout_;
  std::unique_ptr<Estimator> estimator_;
  std::unique_ptr<MotionModel> motion_model_;
  FuserOptions options_;
  bool initialized_ = false;
  std::deque<Step> history_;
  std::deque<Measurement> measurements_;   // queued and fused within the history
  std::deque<Input> inputs_;
};

}  // namespace sac_localization

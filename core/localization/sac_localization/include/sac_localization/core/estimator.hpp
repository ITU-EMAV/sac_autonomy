// Estimator: the filter's engine. A plugin (pluginlib, base class sac_localization::Estimator),
// chosen in the config:
//   estimator:
//     type: ekf        # ekf | ukf | ...
//     ...              # the engine's own parameters (e.g. ukf alpha, beta, kappa)
//
// An estimator only sees State, MotionModel and Measurement, so any engine works with any
// model and any sensor. Timing (queues, late measurements) is the Fuser's job, not the
// estimator's.

#pragma once

#include <string>

#include "sac_localization/core/measurement.hpp"
#include "sac_localization/core/motion_model.hpp"
#include "sac_localization/core/params.hpp"
#include "sac_localization/core/state.hpp"

namespace sac_localization
{

class Estimator
{
public:
  virtual ~Estimator() = default;

  /// Parameters under "estimator." in the config.
  virtual void initialize(const Params & params) = 0;

  /// Moves the belief forward by dt with the motion model.
  virtual void predict(Belief & belief, double dt, const MotionModel & model, const Inputs & inputs) = 0;

  /// Fuses one measurement at the belief's time. Rejects it (and leaves the belief as it
  /// was) when its innovation is beyond the measurement's rejection threshold.
  virtual UpdateResult update(Belief & belief, const Measurement & measurement) = 0;
};

}  // namespace sac_localization

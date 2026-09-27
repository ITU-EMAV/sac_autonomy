// Motion model: how the state evolves between measurements. A plugin (pluginlib, base class
// sac_localization::MotionModel), chosen in the config:
//   motion_model:
//     type: constant_acceleration   # constant_acceleration | kinematic_bicycle | imu_driven
//     ...                           # the model's parameters, e.g. its process noise
//
// Blocks a model does not move (biases, scale factors) stay constant during prediction and
// only get their random-walk noise in processNoise().

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "sac_localization/core/measurement.hpp"
#include "sac_localization/core/params.hpp"
#include "sac_localization/core/state.hpp"

namespace sac_localization
{

class MotionModel
{
public:
  virtual ~MotionModel() = default;

  /// Parameters under "motion_model." in the config.
  virtual void initialize(const Params & params) = 0;

  /// Blocks the model needs besides the core ones.
  virtual void addStates(StateLayoutBuilder & builder) const { (void)builder; }

  /// Sources of the inputs it reads (e.g. {"middle_imu"}); empty for models without inputs.
  virtual std::vector<std::string> inputs() const { return {}; }

  /// x(t + dt) = f(x(t), u). dt is short (the fuser splits long predictions).
  virtual State predict(const State & x, double dt, const Inputs & u) const = 0;

  /// Process noise Q over dt, in the tangent space (tangentSize() x tangentSize()),
  /// including the random walk of every block (biases too).
  virtual Eigen::MatrixXd processNoise(const State & x, double dt, const Inputs & u) const = 0;

  /// Analytic Jacobian df/dδx. Return false and the estimator differentiates numerically.
  virtual bool jacobian(const State & x, double dt, const Inputs & u, Eigen::MatrixXd & F) const
  {
    (void)x;
    (void)dt;
    (void)u;
    (void)F;
    return false;
  }
};

}  // namespace sac_localization

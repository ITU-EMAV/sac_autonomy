// Numerical Jacobians on the state manifold (central differences in the tangent space), for
// models without analytic ones, and to test the analytic ones.

#pragma once

#include <Eigen/Dense>

#include "sac_localization/core/measurement.hpp"
#include "sac_localization/core/motion_model.hpp"
#include "sac_localization/core/state.hpp"

namespace sac_localization
{

/// dh/dδx of a measurement model at x (dimension x tangentSize).
Eigen::MatrixXd numericMeasurementJacobian(const MeasurementModel & model, const State & x, double step = 1e-6);

/// d f(x ⊞ δ) ⊟ f(x) / dδ of a motion model at x (tangentSize x tangentSize).
Eigen::MatrixXd numericMotionJacobian(
  const MotionModel & model, const State & x, double dt, const Inputs & inputs, double step = 1e-6);

/// The model's analytic Jacobian, or the numeric one when it has none.
Eigen::MatrixXd measurementJacobian(const MeasurementModel & model, const State & x);
Eigen::MatrixXd motionJacobian(const MotionModel & model, const State & x, double dt, const Inputs & inputs);

/// Makes a covariance symmetric (numerical drift).
inline void symmetrize(Eigen::MatrixXd & covariance)
{
  covariance = 0.5 * (covariance + covariance.transpose());
}

}  // namespace sac_localization

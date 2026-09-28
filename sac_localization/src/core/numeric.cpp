#include "sac_localization/core/numeric.hpp"

namespace sac_localization
{

Eigen::MatrixXd numericMeasurementJacobian(const MeasurementModel & model, const State & x, double step)
{
  const int n = x.layout().tangentSize();
  Eigen::MatrixXd H(model.dimension(), n);
  Eigen::VectorXd e = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    e(i) = step;
    const Eigen::VectorXd plus = model.predict(x.boxplus(e));
    e(i) = -step;
    const Eigen::VectorXd minus = model.predict(x.boxplus(e));
    e(i) = 0.0;
    H.col(i) = model.residual(plus, minus) / (2.0 * step);
  }
  return H;
}

Eigen::MatrixXd numericMotionJacobian(
  const MotionModel & model, const State & x, double dt, const Inputs & inputs, double step)
{
  const int n = x.layout().tangentSize();
  Eigen::MatrixXd F(n, n);
  Eigen::VectorXd e = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    e(i) = step;
    const State plus = model.predict(x.boxplus(e), dt, inputs);
    e(i) = -step;
    const State minus = model.predict(x.boxplus(e), dt, inputs);
    e(i) = 0.0;
    F.col(i) = plus.boxminus(minus) / (2.0 * step);
  }
  return F;
}

Eigen::MatrixXd measurementJacobian(const MeasurementModel & model, const State & x)
{
  Eigen::MatrixXd H;
  if (model.jacobian(x, H)) {
    return H;
  }
  return numericMeasurementJacobian(model, x);
}

Eigen::MatrixXd motionJacobian(const MotionModel & model, const State & x, double dt, const Inputs & inputs)
{
  Eigen::MatrixXd F;
  if (model.jacobian(x, dt, inputs, F)) {
    return F;
  }
  return numericMotionJacobian(model, x, dt, inputs);
}

}  // namespace sac_localization

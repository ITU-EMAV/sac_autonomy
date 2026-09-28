// Unscented Kalman filter on the state manifold: no Jacobians; sigma points x ⊞ ±columns of
// sqrt((n + lambda) P) go through f and h, and means are found on the manifold (⊞ / ⊟).
//
// Parameters (estimator.*), the usual scaled unscented transform:
//   alpha  spread of the sigma points (default 1e-3, as in robot_localization)
//   beta   prior knowledge of the distribution (2: Gaussian)
//   kappa  secondary scaling (0)

#pragma once

#include <vector>

#include "sac_localization/core/estimator.hpp"

namespace sac_localization
{

class Ukf : public Estimator
{
public:
  void initialize(const Params & params) override;
  void predict(Belief & belief, double dt, const MotionModel & model, const Inputs & inputs) override;
  UpdateResult update(Belief & belief, const Measurement & measurement) override;

private:
  struct SigmaPoints
  {
    std::vector<State> points;
    std::vector<double> mean_weights;
    std::vector<double> covariance_weights;
  };
  SigmaPoints sigmaPoints(const Belief & belief) const;
  /// Weighted mean of states on the manifold (iterated from the first point).
  State mean(const std::vector<State> & points, const std::vector<double> & weights) const;

  double alpha_ = 1e-3;
  double beta_ = 2.0;
  double kappa_ = 0.0;
};

}  // namespace sac_localization

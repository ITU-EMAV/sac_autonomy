// Extended Kalman filter on the state manifold (an error-state EKF: corrections are applied
// with ⊞), and its iterated version.
//
// Parameters (estimator.*):
//   iterations            update iterations: 1 is the EKF, more is the iterated EKF (IEKF),
//                         which relinearizes around its own estimate; better when h(x) bends
//                         within the uncertainty (e.g. antenna lever arms with an uncertain yaw)
//   iteration_tolerance   stop iterating when the correction changes less than this

#pragma once

#include "sac_localization/core/estimator.hpp"

namespace sac_localization
{

class Ekf : public Estimator
{
public:
  void initialize(const Params & params) override;
  void predict(Belief & belief, double dt, const MotionModel & model, const Inputs & inputs) override;
  UpdateResult update(Belief & belief, const Measurement & measurement) override;

protected:
  int default_iterations_ = 1;
  int iterations_ = 1;
  double tolerance_ = 1e-6;
};

/// The same engine with 5 iterations by default.
class Iekf : public Ekf
{
public:
  Iekf() { default_iterations_ = 5; }
};

}  // namespace sac_localization

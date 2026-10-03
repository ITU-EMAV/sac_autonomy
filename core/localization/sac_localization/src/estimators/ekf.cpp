#include "sac_localization/estimators/ekf.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "sac_localization/core/numeric.hpp"

namespace sac_localization
{

void Ekf::initialize(const Params & params)
{
  iterations_ = std::max(1, params.getInt("iterations", default_iterations_));
  tolerance_ = params.getDouble("iteration_tolerance", 1e-6);
}

void Ekf::predict(Belief & belief, double dt, const MotionModel & model, const Inputs & inputs)
{
  const Eigen::MatrixXd F = motionJacobian(model, belief.state, dt, inputs);
  const Eigen::MatrixXd Q = model.processNoise(belief.state, dt, inputs);
  belief.state = model.predict(belief.state, dt, inputs);
  belief.covariance = F * belief.covariance * F.transpose() + Q;
  symmetrize(belief.covariance);
}

UpdateResult Ekf::update(Belief & belief, const Measurement & m)
{
  UpdateResult result;
  result.source = m.source;
  result.stamp = m.stamp;

  const State x0 = belief.state;
  const Eigen::MatrixXd & P = belief.covariance;
  const int n = x0.layout().tangentSize();

  State xi = x0;
  Eigen::MatrixXd H, K;
  for (int iteration = 0; iteration < iterations_; ++iteration) {
    const Eigen::VectorXd y = m.model->residual(m.z, m.model->predict(xi));
    H = measurementJacobian(*m.model, xi);
    const Eigen::MatrixXd S = H * P * H.transpose() + m.R;
    const Eigen::LLT<Eigen::MatrixXd> llt(S);
    if (llt.info() != Eigen::Success) {
      result.reason = "innovation covariance not positive definite";
      return result;
    }
    if (iteration == 0) {
      result.innovation = y;
      result.mahalanobis = std::sqrt(std::max(0.0, y.dot(llt.solve(y))));
      if (!std::isfinite(result.mahalanobis) || result.mahalanobis > m.rejection_threshold) {
        result.reason = "outlier";
        return result;
      }
    }
    // Iterated update: linearized at xi, but always a correction of x0
    const Eigen::VectorXd dx = xi.boxminus(x0);
    K = llt.solve(H * P).transpose();  // P H^T S^-1 (S is symmetric)
    const State next = x0.boxplus(K * (y + H * dx));
    const double change = next.boxminus(xi).norm();
    xi = next;
    if (change < tolerance_) {
      break;
    }
  }

  // Joseph form keeps the covariance symmetric and positive
  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(n, n) - K * H;
  Eigen::MatrixXd covariance = IKH * P * IKH.transpose() + K * m.R * K.transpose();
  symmetrize(covariance);
  belief.covariance = std::move(covariance);
  belief.state = xi;
  result.accepted = true;
  return result;
}

}  // namespace sac_localization

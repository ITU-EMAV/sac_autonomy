#include "sac_localization/estimators/ukf.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "sac_localization/core/numeric.hpp"

namespace sac_localization
{

void Ukf::initialize(const Params & params)
{
  alpha_ = params.getDouble("alpha", 1e-3);
  beta_ = params.getDouble("beta", 2.0);
  kappa_ = params.getDouble("kappa", 0.0);
}

Ukf::SigmaPoints Ukf::sigmaPoints(const Belief & belief) const
{
  const int n = belief.state.layout().tangentSize();
  const double lambda = alpha_ * alpha_ * (n + kappa_) - n;
  const double c = n + lambda;

  // Square root of c P; a positive semi-definite P (numerically) falls back to its
  // eigendecomposition
  Eigen::MatrixXd L;
  const Eigen::LLT<Eigen::MatrixXd> llt(c * belief.covariance);
  if (llt.info() == Eigen::Success) {
    L = llt.matrixL();
  } else {
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(c * belief.covariance);
    L = eigen.eigenvectors() * eigen.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal();
  }

  SigmaPoints s;
  s.points.reserve(2 * n + 1);
  s.points.push_back(belief.state);
  for (int i = 0; i < n; ++i) {
    s.points.push_back(belief.state.boxplus(L.col(i)));
  }
  for (int i = 0; i < n; ++i) {
    s.points.push_back(belief.state.boxplus(-L.col(i)));
  }
  s.mean_weights.assign(2 * n + 1, 1.0 / (2.0 * c));
  s.covariance_weights = s.mean_weights;
  s.mean_weights[0] = lambda / c;
  s.covariance_weights[0] = lambda / c + (1.0 - alpha_ * alpha_ + beta_);
  return s;
}

State Ukf::mean(const std::vector<State> & points, const std::vector<double> & weights) const
{
  State m = points.front();
  for (int iteration = 0; iteration < 10; ++iteration) {
    Eigen::VectorXd step = Eigen::VectorXd::Zero(m.layout().tangentSize());
    for (std::size_t i = 0; i < points.size(); ++i) {
      step += weights[i] * points[i].boxminus(m);
    }
    m = m.boxplus(step);
    if (step.norm() < 1e-10) {
      break;
    }
  }
  return m;
}

void Ukf::predict(Belief & belief, double dt, const MotionModel & model, const Inputs & inputs)
{
  const Eigen::MatrixXd Q = model.processNoise(belief.state, dt, inputs);
  SigmaPoints s = sigmaPoints(belief);
  for (State & p : s.points) {
    p = model.predict(p, dt, inputs);
  }
  const State m = mean(s.points, s.mean_weights);
  Eigen::MatrixXd P = Q;
  for (std::size_t i = 0; i < s.points.size(); ++i) {
    const Eigen::VectorXd d = s.points[i].boxminus(m);
    P.noalias() += s.covariance_weights[i] * d * d.transpose();
  }
  symmetrize(P);
  belief.state = m;
  belief.covariance = std::move(P);
}

UpdateResult Ukf::update(Belief & belief, const Measurement & meas)
{
  UpdateResult result;
  result.source = meas.source;
  result.stamp = meas.stamp;

  const SigmaPoints s = sigmaPoints(belief);
  const std::size_t count = s.points.size();
  std::vector<Eigen::VectorXd> z(count);
  for (std::size_t i = 0; i < count; ++i) {
    z[i] = meas.model->predict(s.points[i]);
  }
  // Mean in measurement space through the model's residual (angles wrap)
  Eigen::VectorXd z_mean = z.front();
  for (int iteration = 0; iteration < 5; ++iteration) {
    Eigen::VectorXd step = Eigen::VectorXd::Zero(z_mean.size());
    for (std::size_t i = 0; i < count; ++i) {
      step += s.mean_weights[i] * meas.model->residual(z[i], z_mean);
    }
    z_mean += step;
    if (step.norm() < 1e-12) {
      break;
    }
  }

  const int n = belief.state.layout().tangentSize();
  const int dim = static_cast<int>(z_mean.size());
  Eigen::MatrixXd Pzz = meas.R;
  Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(n, dim);
  for (std::size_t i = 0; i < count; ++i) {
    const Eigen::VectorXd r = meas.model->residual(z[i], z_mean);
    const Eigen::VectorXd d = s.points[i].boxminus(belief.state);
    Pzz.noalias() += s.covariance_weights[i] * r * r.transpose();
    Pxz.noalias() += s.covariance_weights[i] * d * r.transpose();
  }
  symmetrize(Pzz);

  const Eigen::LLT<Eigen::MatrixXd> llt(Pzz);
  if (llt.info() != Eigen::Success) {
    result.reason = "innovation covariance not positive definite";
    return result;
  }
  const Eigen::VectorXd y = meas.model->residual(meas.z, z_mean);
  result.innovation = y;
  result.mahalanobis = std::sqrt(std::max(0.0, y.dot(llt.solve(y))));
  if (!std::isfinite(result.mahalanobis) || result.mahalanobis > meas.rejection_threshold) {
    result.reason = "outlier";
    return result;
  }

  const Eigen::MatrixXd K = llt.solve(Pxz.transpose()).transpose();  // Pxz Pzz^-1
  belief.state = belief.state.boxplus(K * y);
  Eigen::MatrixXd P = belief.covariance - K * Pzz * K.transpose();
  symmetrize(P);
  belief.covariance = std::move(P);
  result.accepted = true;
  return result;
}

}  // namespace sac_localization

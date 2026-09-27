// What sensors give the filter: measurements (fused in an update) and inputs (read by a
// motion model during prediction, e.g. an IMU's rates for the imu_driven model).

#pragma once

#include <limits>
#include <map>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "sac_localization/core/state.hpp"

namespace sac_localization
{

/// z = h(x) + noise. A model knows how the state produces a measurement: which blocks it
/// reads, sensor mounting (lever arms, rotations), biases. Standard models are in
/// measurement_models.hpp; adapters combine and configure them.
class MeasurementModel
{
public:
  virtual ~MeasurementModel() = default;

  virtual int dimension() const = 0;

  /// The expected measurement h(x).
  virtual Eigen::VectorXd predict(const State & x) const = 0;

  /// z - h(x). Override where plain subtraction is wrong, e.g. wrapping angles.
  virtual Eigen::VectorXd residual(const Eigen::VectorXd & z, const Eigen::VectorXd & expected) const
  {
    return z - expected;
  }

  /// Analytic Jacobian dh/dδx (dimension() x tangentSize()). Return false and estimators
  /// that need one (EKF) differentiate predict() numerically on the manifold.
  virtual bool jacobian(const State & x, Eigen::MatrixXd & H) const
  {
    (void)x;
    (void)H;
    return false;
  }
};

struct Measurement
{
  Stamp stamp = 0;
  std::string source;  // the sensor's name in the config
  Eigen::VectorXd z;
  Eigen::MatrixXd R;   // noise covariance of z
  std::shared_ptr<const MeasurementModel> model;
  /// Rejected when the innovation's Mahalanobis distance is larger [sigma]
  double rejection_threshold = std::numeric_limits<double>::infinity();
};

/// A control input: held from its stamp until the next one of the same source.
struct Input
{
  Stamp stamp = 0;
  std::string source;
  Eigen::VectorXd u;
  Eigen::MatrixXd covariance;
};

/// The latest input of each source, as the motion model sees them during a prediction.
class Inputs
{
public:
  const Input * get(const std::string & source) const
  {
    auto it = latest_.find(source);
    return it == latest_.end() ? nullptr : &it->second;
  }
  void set(const Input & input) { latest_[input.source] = input; }

private:
  std::map<std::string, Input> latest_;
};

/// What happened to one measurement, for diagnostics and tuning.
struct UpdateResult
{
  std::string source;
  Stamp stamp = 0;
  bool accepted = false;
  Eigen::VectorXd innovation;  // z - h(x) before the update
  double mahalanobis = 0.0;    // innovation size in standard deviations
  std::string reason;          // why it was rejected: "outlier", "too late", ...
};

}  // namespace sac_localization

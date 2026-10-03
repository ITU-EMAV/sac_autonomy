// Each engine localizes a car driving a circle from two GNSS antennas, the wheel speed, a
// gyro and the non-holonomic constraint, starting 1 m and 17 degrees off.

#include <gtest/gtest.h>

#include <memory>
#include <random>
#include <string>

#include "map_params.hpp"
#include "sac_localization/core/fuser.hpp"
#include "sac_localization/core/measurement_models.hpp"
#include "sac_localization/core/so3.hpp"
#include "sac_localization/estimators/ekf.hpp"
#include "sac_localization/estimators/ukf.hpp"
#include "sac_localization/motion_models/motion_models.hpp"

using namespace sac_localization;

namespace
{
constexpr double kWheelBase = 1.873;
constexpr double kSpeed = 5.0;
constexpr double kYawRate = 0.2;

struct Scenario
{
  std::shared_ptr<const StateLayout> layout;
  std::shared_ptr<const MeasurementModel> antenna1, antenna2, rear_velocity, gyro;

  Scenario()
  {
    StateLayoutBuilder b;
    layout = b.build();
    antenna1 = std::make_shared<PositionModel>(Eigen::Vector3d(0.45, -0.35, 1.6));
    antenna2 = std::make_shared<PositionModel>(Eigen::Vector3d(-0.45, 0.35, 1.6));
    Eigen::Isometry3d rear = Eigen::Isometry3d::Identity();
    rear.translation() = Eigen::Vector3d(-kWheelBase / 2, 0.0, 0.0);
    // forward and lateral velocity of the rear axle (lateral is the non-holonomic constraint)
    rear_velocity = std::make_shared<SubsetModel>(std::make_shared<BodyVelocityModel>(rear), std::vector<int>{0, 1});
    gyro = std::make_shared<SubsetModel>(
      std::make_shared<AngularVelocityModel>(Eigen::Quaterniond::Identity()), std::vector<int>{2});
  }

  /// The true state at time t: driving a circle
  State truth(double t) const
  {
    State x(layout);
    const double yaw = 0.5 + kYawRate * t;
    const double radius = kSpeed / kYawRate;  // of the rear axle
    const Eigen::Vector3d rear(radius * std::sin(yaw), -radius * std::cos(yaw), 0.0);
    x.rotation(blocks::kOrientation) = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    x.vector(blocks::kPosition) = rear + x.orientation() * Eigen::Vector3d(kWheelBase / 2, 0.0, 0.0);
    x.vector(blocks::kLinearVelocity) = Eigen::Vector3d(kSpeed, kYawRate * kWheelBase / 2, 0.0);
    x.vector(blocks::kAngularVelocity) = Eigen::Vector3d(0.0, 0.0, kYawRate);
    x.vector(blocks::kLinearAcceleration) = Eigen::Vector3d(0.0, kSpeed * kYawRate, 0.0);
    return x;
  }
};

Measurement measure(
  const std::shared_ptr<const MeasurementModel> & model, const State & truth, double sigma, Stamp stamp,
  std::mt19937 & random, const std::string & source)
{
  std::normal_distribution<double> normal(0.0, sigma);
  Measurement m;
  m.stamp = stamp;
  m.source = source;
  m.model = model;
  m.z = model->predict(truth);
  for (int i = 0; i < m.z.size(); ++i) {
    m.z(i) += normal(random);
  }
  m.R = Eigen::MatrixXd::Identity(m.z.size(), m.z.size()) * sigma * sigma;
  m.rejection_threshold = 10.0;
  return m;
}

void localizes(std::shared_ptr<Estimator> estimator, const std::string & name)
{
  const Scenario s;
  MapParams params;
  estimator->initialize(params);
  auto model = std::make_shared<ConstantAcceleration>();
  model->initialize(params);
  Fuser fuser(s.layout, estimator, model, FuserOptions{});

  // Start 1 m and 0.3 rad off
  Belief start{0, s.truth(0.0), Eigen::MatrixXd::Identity(s.layout->tangentSize(), s.layout->tangentSize())};
  Eigen::VectorXd error = Eigen::VectorXd::Zero(s.layout->tangentSize());
  error.head<3>() = Eigen::Vector3d(0.8, -0.6, 0.0);
  error(5) = 0.3;
  start.state = start.state.boxplus(error);
  start.covariance.diagonal().segment<3>(3) = Eigen::Vector3d(0.01, 0.01, 0.25);
  fuser.reset(start);

  std::mt19937 random(7);
  for (int k = 1; k <= 300; ++k) {  // 30 s at 10 Hz
    const double t = 0.1 * k;
    const Stamp stamp = fromSeconds(t);
    const State truth = s.truth(t);
    fuser.add(measure(s.antenna1, truth, 0.3, stamp, random, "gnss1"));
    fuser.add(measure(s.antenna2, truth, 0.3, stamp, random, "gnss2"));
    fuser.add(measure(s.rear_velocity, truth, 0.05, stamp, random, "wheels"));
    fuser.add(measure(s.gyro, truth, 0.01, stamp, random, "gyro"));
    fuser.update(stamp);
  }
  const State truth = s.truth(30.0);
  const State & estimate = fuser.belief().state;
  const double position_error = (estimate.position() - truth.position()).head<2>().norm();
  const double yaw_error = std::abs(wrapAngle(yawOf(estimate.orientation()) - yawOf(truth.orientation())));
  EXPECT_LT(position_error, 0.3) << name;
  EXPECT_LT(yaw_error, 0.03) << name;
}
}  // namespace

TEST(Estimators, EkfLocalizes) { localizes(std::make_shared<Ekf>(), "ekf"); }
TEST(Estimators, IekfLocalizes) { localizes(std::make_shared<Iekf>(), "iekf"); }
TEST(Estimators, UkfLocalizes) { localizes(std::make_shared<Ukf>(), "ukf"); }

TEST(Estimators, RejectsOutliers)
{
  const Scenario s;
  Ekf ekf;
  MapParams params;
  ekf.initialize(params);
  Belief b{0, s.truth(0.0), Eigen::MatrixXd::Identity(s.layout->tangentSize(), s.layout->tangentSize()) * 0.01};
  Measurement m;
  m.model = s.antenna1;
  m.z = s.antenna1->predict(b.state) + Eigen::Vector3d(50.0, 0.0, 0.0);
  m.R = Eigen::Matrix3d::Identity() * 0.09;
  m.rejection_threshold = 5.0;
  const State before = b.state;
  const UpdateResult r = ekf.update(b, m);
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.reason, "outlier");
  EXPECT_TRUE(b.state.boxminus(before).isZero());
}

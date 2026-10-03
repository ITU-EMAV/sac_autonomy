// Every measurement model: the analytic Jacobian (where there is one) matches the numeric
// one, and the models give the physically expected values.

#include <gtest/gtest.h>

#include <memory>
#include <random>

#include "sac_localization/core/measurement_models.hpp"
#include "sac_localization/core/numeric.hpp"
#include "sac_localization/core/so3.hpp"

using namespace sac_localization;

namespace
{
State randomState(unsigned seed)
{
  StateLayoutBuilder b;
  b.add("imu/gyro_bias", BlockKind::kVector, 3);
  b.add("imu/accel_bias", BlockKind::kVector, 3);
  b.add("mag/offset", BlockKind::kVector, 3);
  State x(b.build());
  std::mt19937 random(seed);
  std::normal_distribution<double> normal(0.0, 1.0);
  Eigen::VectorXd d(x.layout().tangentSize());
  for (int i = 0; i < d.size(); ++i) {
    d(i) = normal(random);
  }
  return x.boxplus(d);
}

Eigen::Isometry3d mount()
{
  Eigen::Isometry3d m = Eigen::Isometry3d::Identity();
  m.translation() = Eigen::Vector3d(-0.45, 0.35, 1.6);
  m.linear() = expSO3(Eigen::Vector3d(0.1, -0.2, 0.3)).toRotationMatrix();
  return m;
}

void expectJacobianMatches(const MeasurementModel & model, const std::string & name)
{
  for (unsigned seed = 1; seed <= 5; ++seed) {
    const State x = randomState(seed);
    const Eigen::MatrixXd numeric = numericMeasurementJacobian(model, x);
    Eigen::MatrixXd analytic;
    if (model.jacobian(x, analytic)) {
      EXPECT_TRUE(analytic.isApprox(numeric, 1e-5)) << name << "\nanalytic\n" << analytic << "\nnumeric\n" << numeric;
    }
    EXPECT_TRUE(numeric.allFinite()) << name;
  }
}
}  // namespace

TEST(Models, JacobiansMatchNumeric)
{
  const Eigen::Isometry3d m = mount();
  const Eigen::Quaterniond r(m.linear());
  auto body_velocity = std::make_shared<BodyVelocityModel>(m);
  auto gyro = std::make_shared<AngularVelocityModel>(r, "imu/gyro_bias");
  expectJacobianMatches(PositionModel(m.translation()), "position");
  expectJacobianMatches(OrientationModel(r), "orientation");
  expectJacobianMatches(YawModel(0.2), "yaw");
  expectJacobianMatches(*body_velocity, "body velocity");
  expectJacobianMatches(WorldVelocityModel(m.translation()), "world velocity");
  expectJacobianMatches(*gyro, "gyro");
  expectJacobianMatches(SpecificForceModel(m, "imu/accel_bias"), "accelerometer");
  expectJacobianMatches(MagneticFieldModel(r, {0.2, 0.0, -0.4}, "mag/offset"), "magnetometer");
  expectJacobianMatches(SubsetModel(body_velocity, {0, 2}), "subset");
  expectJacobianMatches(StackedModel({body_velocity, gyro}), "stacked");
}

TEST(Models, PhysicalValues)
{
  StateLayoutBuilder b;
  State x(b.build());
  // At rest and level an accelerometer reads +g upwards
  const Eigen::VectorXd f = SpecificForceModel(Eigen::Isometry3d::Identity()).predict(x);
  EXPECT_TRUE(f.isApprox(Eigen::Vector3d(0, 0, 9.80665), 1e-12));
  // Turning left at 1 rad/s, a point 1 m behind the origin moves to the right: w x r
  x.vector(blocks::kAngularVelocity) = Eigen::Vector3d(0, 0, 1);
  Eigen::Isometry3d behind = Eigen::Isometry3d::Identity();
  behind.translation() = Eigen::Vector3d(-1, 0, 0);
  EXPECT_TRUE(BodyVelocityModel(behind).predict(x).isApprox(Eigen::Vector3d(0, -1, 0), 1e-12));
  // An antenna 1 m to the left of a car facing north (yaw 90 deg) is 1 m west
  x.rotation(blocks::kOrientation) = expSO3(Eigen::Vector3d(0, 0, M_PI / 2));
  EXPECT_TRUE(PositionModel({0, 1, 0}).predict(x).isApprox(Eigen::Vector3d(-1, 0, 0), 1e-12));
  // Yaw residuals wrap
  Eigen::VectorXd a(1), e(1);
  a << 3.1;
  e << -3.1;
  EXPECT_NEAR(YawModel().residual(a, e)(0), 6.2 - 2 * M_PI, 1e-12);
}

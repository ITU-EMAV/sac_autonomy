// The fuser gives the same result whether a measurement arrives on time or late (within the
// history), and drops ones older than the history.

#include <gtest/gtest.h>

#include <memory>

#include "map_params.hpp"
#include "sac_localization/core/fuser.hpp"
#include "sac_localization/core/measurement_models.hpp"
#include "sac_localization/estimators/ekf.hpp"
#include "sac_localization/motion_models/motion_models.hpp"

using namespace sac_localization;

namespace
{
std::unique_ptr<Fuser> makeFuser()
{
  StateLayoutBuilder b;
  auto layout = b.build();
  MapParams params;
  auto ekf = std::make_shared<Ekf>();
  ekf->initialize(params);
  auto model = std::make_shared<ConstantAcceleration>();
  model->initialize(params);
  auto fuser = std::make_unique<Fuser>(layout, ekf, model, FuserOptions{});
  fuser->reset(Belief{0, State(layout), Eigen::MatrixXd::Identity(layout->tangentSize(), layout->tangentSize())});
  return fuser;
}

Measurement position(double t, double x)
{
  Measurement m;
  m.stamp = fromSeconds(t);
  m.source = "gnss";
  m.model = std::make_shared<PositionModel>(Eigen::Vector3d::Zero());
  m.z = Eigen::Vector3d(x, 0.5 * x, 0.0);
  m.R = Eigen::Matrix3d::Identity() * 0.1;
  return m;
}

Measurement speed(double t, double v)
{
  Measurement m;
  m.stamp = fromSeconds(t);
  m.source = "wheels";
  m.model = std::make_shared<SubsetModel>(
    std::make_shared<BodyVelocityModel>(Eigen::Isometry3d::Identity()), std::vector<int>{0});
  m.z = Eigen::VectorXd::Constant(1, v);
  m.R = Eigen::MatrixXd::Identity(1, 1) * 0.01;
  return m;
}
}  // namespace

TEST(Fuser, LateMeasurementGivesTheSameResult)
{
  auto on_time = makeFuser();
  auto late = makeFuser();
  // Speed at 50 Hz; GNSS at 10 Hz, arriving 0.2 s late for `late`
  for (int k = 1; k <= 100; ++k) {
    const double t = 0.02 * k;
    on_time->add(speed(t, 2.0));
    late->add(speed(t, 2.0));
    if (k % 5 == 0) {
      on_time->add(position(t, 2.0 * t));
    }
    if (k % 5 == 0 && k > 10) {
      late->add(position(t - 0.2, 2.0 * (t - 0.2)));
    }
    on_time->update(fromSeconds(t));
    late->update(fromSeconds(t));
  }
  // The last 0.2 s of GNSS
  late->add(position(1.9, 3.8));
  late->add(position(2.0, 4.0));
  on_time->update(fromSeconds(2.0));
  late->update(fromSeconds(2.0));
  // The same up to rounding (the late fixes' values differ in the last bits: 0.3 - 0.2)
  EXPECT_LT(late->belief().state.boxminus(on_time->belief().state).norm(), 1e-6);
  EXPECT_TRUE(late->belief().covariance.isApprox(on_time->belief().covariance, 1e-6));
}

TEST(Fuser, DropsMeasurementsOlderThanTheHistory)
{
  auto fuser = makeFuser();
  for (int k = 1; k <= 100; ++k) {
    fuser->add(speed(0.02 * k, 1.0));
    fuser->update(fromSeconds(0.02 * k));
  }
  fuser->add(position(0.5, 0.0));  // 1.5 s old, the history is 1 s
  const auto results = fuser->update(fromSeconds(2.02));
  ASSERT_EQ(results.size(), 1u);
  EXPECT_FALSE(results[0].accepted);
  EXPECT_EQ(results[0].reason, "too late");
}

TEST(Fuser, ClearsWhenTheClockJumpsBack)
{
  auto fuser = makeFuser();
  fuser->update(fromSeconds(10.0));
  fuser->update(fromSeconds(2.0));
  EXPECT_FALSE(fuser->initialized());
}

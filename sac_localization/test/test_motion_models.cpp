// The motion models: the dynamic bicycle is stable at low speed and slips as a car does.

#include <gtest/gtest.h>

#include <cmath>

#include "map_params.hpp"
#include "sac_localization/core/numeric.hpp"
#include "sac_localization/motion_models/motion_models.hpp"

using namespace sac_localization;

namespace
{
Inputs wheels(double speed, double steering)
{
  Input input;
  input.source = "wheels";
  input.u = Eigen::Vector2d(speed, steering);
  Inputs inputs;
  inputs.set(input);
  return inputs;
}

DynamicBicycle model()
{
  MapParams params;
  params.strings["input"] = "wheels";
  DynamicBicycle m;
  m.initialize(params);
  return m;
}

/// Runs the model for `seconds` in 10 ms steps
State drive(const DynamicBicycle & m, double speed, double steering, double seconds)
{
  StateLayoutBuilder b;
  State x(b.build());
  const Inputs u = wheels(speed, steering);
  for (int i = 0; i < static_cast<int>(seconds / 0.01); ++i) {
    x = m.predict(x, 0.01, u);
  }
  return x;
}
}  // namespace

TEST(DynamicBicycle, StableAndKinematicAtLowSpeed)
{
  const DynamicBicycle m = model();
  for (double speed : {0.0, 0.3, 1.0, 1.9}) {
    const State x = drive(m, speed, 0.5, 5.0);
    EXPECT_TRUE(x.linearVelocity().allFinite() && x.angularVelocity().allFinite()) << speed;
    if (speed < 0.5) {
      // Fully kinematic: yaw rate v tan(steering) / wheel base
      EXPECT_NEAR(x.angularVelocity().z(), speed * std::tan(0.5) / (1.124 + 0.749), 1e-9) << speed;
    }
  }
}

TEST(DynamicBicycle, SlipsInCorners)
{
  const DynamicBicycle m = model();
  const double speed = 10.0;
  const double steering = 0.05;  // a left turn, ~3.5 m/s^2 of lateral acceleration
  const State x = drive(m, speed, steering, 5.0);
  const double r = x.angularVelocity().z();
  const double r_kinematic = speed * std::tan(steering) / (1.124 + 0.749);
  // These tyres load-proportional: close to neutral steer, the yaw rate near the kinematic one
  EXPECT_NEAR(r, r_kinematic, 0.05 * r_kinematic);
  // The rear axle slides outwards (to the right) to make its tyres push the car into the turn
  const double rear_lateral = x.linearVelocity().y() - r * (1.124 + 0.749) / 2.0;  // half a wheel base behind
  EXPECT_LT(rear_lateral, -0.05);
  EXPECT_GT(rear_lateral, -1.0);
}

TEST(DynamicBicycle, NumericJacobianIsFinite)
{
  const DynamicBicycle m = model();
  const State x = drive(m, 8.0, 0.1, 2.0);
  const Eigen::MatrixXd F = numericMotionJacobian(m, x, 0.01, wheels(8.0, 0.1));
  EXPECT_TRUE(F.allFinite());
}

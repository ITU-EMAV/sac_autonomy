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

DynamicBicycle model(const std::string & speed_from = "input")
{
  MapParams params;
  params.strings["input"] = "wheels";
  params.strings["speed_from"] = speed_from;
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

TEST(DynamicBicycle, SpeedFromTheStateIgnoresTheWheelSpeed)
{
  // An airborne wheel spins at 10 m/s while the car goes 8 m/s: with speed_from state the
  // prediction keeps the state's speed (the wheel speed is a measurement then)
  const DynamicBicycle m = model("state");
  StateLayoutBuilder b;
  State x(b.build());
  x.vector(blocks::kLinearVelocity) = Eigen::Vector3d(8.0, 0.0, 0.0);
  const Inputs u = wheels(10.0, 0.0);
  for (int i = 0; i < 100; ++i) {
    x = m.predict(x, 0.01, u);
  }
  EXPECT_NEAR(x.linearVelocity().x(), 8.0, 1e-9);
  EXPECT_NEAR(x.position().x(), 8.0, 1e-6);
}

TEST(DynamicBicycle, NumericJacobianIsFinite)
{
  const DynamicBicycle m = model();
  const State x = drive(m, 8.0, 0.1, 2.0);
  const Eigen::MatrixXd F = numericMotionJacobian(m, x, 0.01, wheels(8.0, 0.1));
  EXPECT_TRUE(F.allFinite());
}

TEST(DynamicBicycle, ManualStiffnessAddsNoState)
{
  const DynamicBicycle m = model();
  StateLayoutBuilder b;
  m.addStates(b);
  EXPECT_FALSE(b.build()->has(DynamicBicycle::kStiffnessBlock));
}

TEST(DynamicBicycle, EstimatedStiffnessStartsFromTheConfiguredValues)
{
  for (const std::string what : {"grip", "front_rear"}) {
    MapParams params;
    params.strings["input"] = "wheels";
    params.strings["speed_from"] = "input";
    params.strings["cornering_stiffness_mode"] = "estimate";
    params.strings["cornering_stiffness_estimate"] = what;
    params.doubles["cornering_stiffness_front"] = 30000.0;
    params.doubles["cornering_stiffness_rear"] = 50000.0;
    DynamicBicycle m;
    m.initialize(params);
    StateLayoutBuilder b;
    m.addStates(b);
    const auto layout = b.build();
    ASSERT_TRUE(layout->has(DynamicBicycle::kStiffnessBlock));
    EXPECT_EQ(layout->block(DynamicBicycle::kStiffnessBlock).tangent_size, what == "grip" ? 1 : 2);
    Belief belief{0, State(layout), Eigen::MatrixXd::Identity(layout->tangentSize(), layout->tangentSize())};
    m.initializeBelief(belief);
    const auto p = m.estimatedParameters(belief);
    ASSERT_EQ(p.size(), 2u);
    EXPECT_DOUBLE_EQ(p[0].value, 30000.0);
    EXPECT_DOUBLE_EQ(p[1].value, 50000.0);
    EXPECT_NEAR(p[0].stddev, 30000.0 * 0.5, 1e-6);  // the default uncertainty

    // A factor of 2 in the state predicts as twice the configured stiffness does
    State x = belief.state;
    x.vector(blocks::kLinearVelocity) = Eigen::Vector3d(10.0, 0.1, 0.0);
    x.vector(blocks::kAngularVelocity) = Eigen::Vector3d(0.0, 0.0, 0.2);
    x.vector(DynamicBicycle::kStiffnessBlock).setConstant(std::log(2.0));
    MapParams doubled = params;
    doubled.strings["cornering_stiffness_mode"] = "manual";
    doubled.doubles["cornering_stiffness_front"] = 60000.0;
    doubled.doubles["cornering_stiffness_rear"] = 100000.0;
    DynamicBicycle reference;
    reference.initialize(doubled);
    const Inputs u = wheels(10.0, 0.05);
    EXPECT_TRUE(m.predict(x, 0.01, u).boxminus(reference.predict(x, 0.01, u)).isZero(1e-9)) << what;
  }
}

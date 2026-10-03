#include <gtest/gtest.h>

#include <random>

#include "sac_localization/core/geodesy.hpp"
#include "sac_localization/core/so3.hpp"
#include "sac_localization/core/state.hpp"

using namespace sac_localization;

namespace
{
std::shared_ptr<const StateLayout> layoutWithBias()
{
  StateLayoutBuilder b;
  b.add("imu/gyro_bias", BlockKind::kVector, 3);
  b.add("imu/gyro_bias", BlockKind::kVector, 3);  // twice is fine
  return b.build();
}
}  // namespace

TEST(State, Layout)
{
  auto layout = layoutWithBias();
  EXPECT_EQ(layout->tangentSize(), 18);
  EXPECT_EQ(layout->rotationCount(), 1);
  EXPECT_EQ(layout->vectorStorageSize(), 15);
  EXPECT_TRUE(layout->has("imu/gyro_bias"));
  EXPECT_THROW(layout->block("nope"), std::out_of_range);
  StateLayoutBuilder b;
  EXPECT_THROW(b.add(blocks::kPosition, BlockKind::kVector, 2), std::invalid_argument);
}

TEST(State, BoxplusBoxminusRoundTrip)
{
  auto layout = layoutWithBias();
  std::mt19937 random(1);
  std::normal_distribution<double> normal(0.0, 0.5);
  State x(layout);
  for (int trial = 0; trial < 50; ++trial) {
    Eigen::VectorXd d(layout->tangentSize());
    for (int i = 0; i < d.size(); ++i) {
      d(i) = normal(random);
    }
    const State y = x.boxplus(d);
    EXPECT_TRUE(y.boxminus(x).isApprox(d, 1e-9));
    x = y;
  }
}

TEST(So3, ExpLog)
{
  const Eigen::Vector3d v(0.3, -1.2, 2.0);
  EXPECT_TRUE(logSO3(expSO3(v)).isApprox(v, 1e-12));
  EXPECT_NEAR(yawOf(expSO3(Eigen::Vector3d(0.0, 0.0, 2.5))), 2.5, 1e-12);
  EXPECT_NEAR(wrapAngle(3.0 * M_PI), M_PI, 1e-12);
}

TEST(Geodesy, RoundTripAndHeading)
{
  // Sonoma, as in the simulation (sac_planning/config/sonoma.yaml)
  const MapFrame map(Datum{38.1628083, -122.4579944, 0.0, 0.83});
  for (const Eigen::Vector3d & p :
       {Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(277.88, -135.2, 3.0), Eigen::Vector3d(-500, 400, 40)})
  {
    const Eigen::Vector3d geo = map.toGeodetic(p);
    EXPECT_LT((map.toMap(geo.x(), geo.y(), geo.z()) - p).norm(), 1e-6) << p.transpose();
  }
  // 100 m east of the origin is at -heading in a map turned by heading
  const MapFrame turned(Datum{38.0, -122.0, 0.0, 30.0});
  const Eigen::Vector3d east = MapFrame(Datum{38.0, -122.0, 0.0, 0.0}).toGeodetic({100.0, 0.0, 0.0});
  const Eigen::Vector3d q = turned.toMap(east.x(), east.y(), east.z());
  EXPECT_NEAR(std::atan2(q.y(), q.x()) * 180.0 / M_PI, -30.0, 0.01);
  // One degree of latitude is about 111 km
  EXPECT_NEAR(MapFrame(Datum{38.0, -122.0, 0.0, 0.0}).toMap(39.0, -122.0, 0.0).y(), 111000.0, 300.0);
}

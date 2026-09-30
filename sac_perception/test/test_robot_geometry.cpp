// The car's box from a URDF: shapes placed through the joints, meshes read for their bounds.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>

#include "sac_perception/robot_geometry.hpp"

using namespace sac_perception;

namespace
{
/// A binary STL of one triangle
std::string writeStl(const std::string & path)
{
  std::ofstream out(path, std::ios::binary);
  char header[80] = {};
  out.write(header, 80);
  const uint32_t n = 1;
  out.write(reinterpret_cast<const char *>(&n), 4);
  const float tri[12] = {0, 0, 1, 0, 0, 0, 100, 0, 0, 0, 0, 50};  // normal, then 3 vertices [mm]
  out.write(reinterpret_cast<const char *>(tri), sizeof(tri));
  const uint16_t attributes = 0;
  out.write(reinterpret_cast<const char *>(&attributes), 2);
  return path;
}
}  // namespace

TEST(RobotGeometry, TheCarsBoxFromItsShapes)
{
  const std::string stl = writeStl(testing::TempDir() + "sensor.stl");
  const std::string urdf = R"(<?xml version="1.0"?>
<robot name="car">
  <link name="base_footprint"/>
  <joint name="body_joint" type="fixed">
    <parent link="base_footprint"/><child link="body"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="body">
    <collision><origin xyz="0 0 0.85"/><geometry><box size="2.6 1.6 1.4"/></geometry></collision>
  </link>
  <joint name="wheel_joint" type="continuous">
    <parent link="body"/><child link="wheel"/>
    <origin xyz="1.0 0.8 0.3" rpy="-1.5708 0 0"/><axis xyz="0 0 1"/>
  </joint>
  <link name="wheel">
    <collision><geometry><cylinder radius="0.3" length="0.2"/></geometry></collision>
  </link>
  <joint name="sensor_joint" type="fixed">
    <parent link="body"/><child link="sensor"/>
    <origin xyz="0 0 1.6"/>
  </joint>
  <link name="sensor">
    <visual><geometry><mesh filename="file://)" + stl + R"(" scale="0.001 0.001 0.001"/></geometry></visual>
    <visual><geometry><mesh filename="package://no_such_package/meshes/x.dae"/></geometry></visual>
  </link>
</robot>)";
  const auto geometry = vehicleFromUrdf(urdf, "base_footprint");
  ASSERT_TRUE(geometry);
  const VehicleBox & box = geometry->box;
  EXPECT_TRUE(box.known);
  EXPECT_NEAR(box.min.x(), -1.3f, 1e-3f);
  EXPECT_NEAR(box.max.x(), 1.3f, 1e-3f);    // the wheel (1.0 + 0.3) as far as the body
  EXPECT_NEAR(box.min.y(), -0.8f, 1e-3f);
  EXPECT_NEAR(box.max.y(), 0.9f, 1e-3f);    // the wheel: 0.8 + half its width
  EXPECT_NEAR(box.min.z(), 0.0f, 1e-3f);    // the wheel on the ground
  EXPECT_NEAR(box.max.z(), 1.65f, 1e-3f);   // the mesh: 1.6 m + 50 mm
  ASSERT_EQ(geometry->warnings.size(), 1u);  // the .dae
  std::remove(stl.c_str());
}

TEST(RobotGeometry, NoBaseFrameNoBox)
{
  EXPECT_FALSE(vehicleFromUrdf("<robot name=\"r\"><link name=\"a\"/></robot>", "base_footprint"));
  EXPECT_FALSE(vehicleFromUrdf("not a urdf", "base_footprint"));
}

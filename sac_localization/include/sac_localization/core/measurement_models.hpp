// Standard measurement models: the pieces sensor adapters are built from.
//
// Sensor mounting comes from TF (sac_description): `mount` is the sensor frame in the body
// frame (base_footprint), its translation the lever arm.
//   imu                -> AngularVelocityModel + SpecificForceModel (+ OrientationModel)
//   gnss_position      -> PositionModel (lever arm = antenna)
//   gnss_velocity      -> WorldVelocityModel
//   wheel / odometry   -> BodyVelocityModel (forward speed) + AngularVelocityModel (yaw rate)
//   pose / geo_pose    -> PositionModel + OrientationModel
//   heading            -> YawModel
//   magnetometer       -> MagneticFieldModel
//   nonholonomic       -> BodyVelocityModel (y, z = 0)
//   zero_velocity      -> BodyVelocityModel + AngularVelocityModel (all 0)
// SubsetModel keeps only some components of any model (the `use` list in the config).

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "sac_localization/core/measurement.hpp"

namespace sac_localization
{

/// Position of a point on the vehicle (the sensor) in world: p + R(q) * lever_arm. [3]
class PositionModel : public MeasurementModel
{
public:
  explicit PositionModel(const Eigen::Vector3d & lever_arm);
  int dimension() const override { return 3; }
  const Eigen::Vector3d & leverArm() const { return lever_arm_; }
  Eigen::VectorXd predict(const State & x) const override;
  bool jacobian(const State & x, Eigen::MatrixXd & H) const override;

private:
  Eigen::Vector3d lever_arm_;
};

/// Orientation of the sensor in world, as a rotation vector; the residual is Log(R_z^T R(x)). [3]
class OrientationModel : public MeasurementModel
{
public:
  explicit OrientationModel(const Eigen::Quaterniond & mount_rotation);
  int dimension() const override { return 3; }
  Eigen::VectorXd predict(const State & x) const override;
  Eigen::VectorXd residual(const Eigen::VectorXd & z, const Eigen::VectorXd & expected) const override;

private:
  Eigen::Quaterniond mount_rotation_;
};

/// Yaw of the vehicle plus a fixed offset (e.g. of an antenna baseline); the residual wraps
/// to [-pi, pi]. [1]
class YawModel : public MeasurementModel
{
public:
  explicit YawModel(double offset = 0.0);
  int dimension() const override { return 1; }
  Eigen::VectorXd predict(const State & x) const override;
  Eigen::VectorXd residual(const Eigen::VectorXd & z, const Eigen::VectorXd & expected) const override;

private:
  double offset_;
};

/// Velocity of a point on the vehicle in the sensor frame: R_m^T (v + w x r). [3]
class BodyVelocityModel : public MeasurementModel
{
public:
  explicit BodyVelocityModel(const Eigen::Isometry3d & mount);
  int dimension() const override { return 3; }
  Eigen::VectorXd predict(const State & x) const override;
  bool jacobian(const State & x, Eigen::MatrixXd & H) const override;

private:
  Eigen::Isometry3d mount_;
};

/// Velocity of a point on the vehicle in world: R(q) (v + w x r). [3]
class WorldVelocityModel : public MeasurementModel
{
public:
  explicit WorldVelocityModel(const Eigen::Vector3d & lever_arm);
  int dimension() const override { return 3; }
  Eigen::VectorXd predict(const State & x) const override;

private:
  Eigen::Vector3d lever_arm_;
};

/// Gyro: R_m^T w + bias. The bias block ("<sensor>/gyro_bias") is optional. [3]
class AngularVelocityModel : public MeasurementModel
{
public:
  AngularVelocityModel(const Eigen::Quaterniond & mount_rotation, std::string bias_block = "");
  int dimension() const override { return 3; }
  Eigen::VectorXd predict(const State & x) const override;
  bool jacobian(const State & x, Eigen::MatrixXd & H) const override;

private:
  Eigen::Quaterniond mount_rotation_;
  std::string bias_block_;
};

/// Accelerometer (specific force) at the sensor: R_m^T (a + w' x r + w x (w x r) - R(q)^T g)
/// + bias, g = (0, 0, -9.81). The bias block ("<sensor>/accel_bias") is optional. [3]
class SpecificForceModel : public MeasurementModel
{
public:
  SpecificForceModel(const Eigen::Isometry3d & mount, std::string bias_block = "", double gravity = 9.80665);
  int dimension() const override { return 3; }
  Eigen::VectorXd predict(const State & x) const override;

private:
  Eigen::Isometry3d mount_;
  std::string bias_block_;
  double gravity_;
};

/// Magnetometer: R_m^T R(q)^T m_world, with the local field m_world (declination and
/// inclination included) and an optional hard-iron offset block. [3]
class MagneticFieldModel : public MeasurementModel
{
public:
  MagneticFieldModel(
    const Eigen::Quaterniond & mount_rotation, const Eigen::Vector3d & world_field,
    std::string offset_block = "");
  int dimension() const override { return 3; }
  Eigen::VectorXd predict(const State & x) const override;

private:
  Eigen::Quaterniond mount_rotation_;
  Eigen::Vector3d world_field_;
  std::string offset_block_;
};

/// Keeps the given components of another model (e.g. only the forward speed of a
/// BodyVelocityModel), so adapters fuse exactly what the config's `use` lists.
class SubsetModel : public MeasurementModel
{
public:
  SubsetModel(std::shared_ptr<const MeasurementModel> model, std::vector<int> components);
  int dimension() const override { return static_cast<int>(components_.size()); }
  Eigen::VectorXd predict(const State & x) const override;
  Eigen::VectorXd residual(const Eigen::VectorXd & z, const Eigen::VectorXd & expected) const override;
  bool jacobian(const State & x, Eigen::MatrixXd & H) const override;

private:
  std::shared_ptr<const MeasurementModel> model_;
  std::vector<int> components_;
};

/// Several models stacked into one measurement (fused together, with their cross
/// covariance), e.g. position + orientation of a pose.
class StackedModel : public MeasurementModel
{
public:
  explicit StackedModel(std::vector<std::shared_ptr<const MeasurementModel>> models);
  int dimension() const override;
  Eigen::VectorXd predict(const State & x) const override;
  Eigen::VectorXd residual(const Eigen::VectorXd & z, const Eigen::VectorXd & expected) const override;
  bool jacobian(const State & x, Eigen::MatrixXd & H) const override;

private:
  std::vector<std::shared_ptr<const MeasurementModel>> models_;
};

}  // namespace sac_localization

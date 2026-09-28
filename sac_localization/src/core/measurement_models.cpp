#include "sac_localization/core/measurement_models.hpp"

#include <stdexcept>
#include <utility>

#include "sac_localization/core/so3.hpp"

namespace sac_localization
{

namespace
{
Eigen::MatrixXd zeros(int rows, const State & x)
{
  return Eigen::MatrixXd::Zero(rows, x.layout().tangentSize());
}
int offset(const State & x, const std::string & block)
{
  return x.layout().block(block).tangent_offset;
}
Eigen::Vector3d optionalBlock(const State & x, const std::string & block)
{
  return block.empty() ? Eigen::Vector3d::Zero() : Eigen::Vector3d(x.vector(block));
}
}  // namespace

// ---------------------------------------------------------------- position
PositionModel::PositionModel(const Eigen::Vector3d & lever_arm)
: lever_arm_(lever_arm) {}

Eigen::VectorXd PositionModel::predict(const State & x) const
{
  return x.position() + x.orientation() * lever_arm_;
}

bool PositionModel::jacobian(const State & x, Eigen::MatrixXd & H) const
{
  H = zeros(3, x);
  H.block<3, 3>(0, offset(x, blocks::kPosition)) = Eigen::Matrix3d::Identity();
  // R Exp(d) r ~ R r - R [r]x d
  H.block<3, 3>(0, offset(x, blocks::kOrientation)) =
    -x.orientation().toRotationMatrix() * skew(lever_arm_);
  return true;
}

// ---------------------------------------------------------------- orientation
OrientationModel::OrientationModel(const Eigen::Quaterniond & mount_rotation)
: mount_rotation_(mount_rotation.normalized()) {}

Eigen::VectorXd OrientationModel::predict(const State & x) const
{
  return logSO3(x.orientation() * mount_rotation_);
}

Eigen::VectorXd OrientationModel::residual(const Eigen::VectorXd & z, const Eigen::VectorXd & expected) const
{
  const Eigen::Vector3d zv = z.head<3>();
  const Eigen::Vector3d ev = expected.head<3>();
  return logSO3(expSO3(ev).conjugate() * expSO3(zv));
}

// ---------------------------------------------------------------- yaw
YawModel::YawModel(double offset)
: offset_(offset) {}

Eigen::VectorXd YawModel::predict(const State & x) const
{
  Eigen::VectorXd z(1);
  z(0) = wrapAngle(yawOf(x.orientation()) + offset_);
  return z;
}

Eigen::VectorXd YawModel::residual(const Eigen::VectorXd & z, const Eigen::VectorXd & expected) const
{
  Eigen::VectorXd r(1);
  r(0) = wrapAngle(z(0) - expected(0));
  return r;
}

// ---------------------------------------------------------------- body velocity
BodyVelocityModel::BodyVelocityModel(const Eigen::Isometry3d & mount)
: mount_(mount) {}

Eigen::VectorXd BodyVelocityModel::predict(const State & x) const
{
  const Eigen::Vector3d r = mount_.translation();
  return mount_.rotation().transpose() * (x.linearVelocity() + x.angularVelocity().cross(r));
}

bool BodyVelocityModel::jacobian(const State & x, Eigen::MatrixXd & H) const
{
  const Eigen::Matrix3d rt = mount_.rotation().transpose();
  H = zeros(3, x);
  H.block<3, 3>(0, offset(x, blocks::kLinearVelocity)) = rt;
  // w x r = -[r]x w
  H.block<3, 3>(0, offset(x, blocks::kAngularVelocity)) = -rt * skew(mount_.translation());
  return true;
}

// ---------------------------------------------------------------- world velocity
WorldVelocityModel::WorldVelocityModel(const Eigen::Vector3d & lever_arm)
: lever_arm_(lever_arm) {}

Eigen::VectorXd WorldVelocityModel::predict(const State & x) const
{
  return x.orientation() * (x.linearVelocity() + x.angularVelocity().cross(lever_arm_));
}

// ---------------------------------------------------------------- gyro
AngularVelocityModel::AngularVelocityModel(const Eigen::Quaterniond & mount_rotation, std::string bias_block)
: mount_rotation_(mount_rotation.normalized()), bias_block_(std::move(bias_block)) {}

Eigen::VectorXd AngularVelocityModel::predict(const State & x) const
{
  return mount_rotation_.conjugate() * x.angularVelocity() + optionalBlock(x, bias_block_);
}

bool AngularVelocityModel::jacobian(const State & x, Eigen::MatrixXd & H) const
{
  H = zeros(3, x);
  H.block<3, 3>(0, offset(x, blocks::kAngularVelocity)) = mount_rotation_.conjugate().toRotationMatrix();
  if (!bias_block_.empty()) {
    H.block<3, 3>(0, offset(x, bias_block_)) = Eigen::Matrix3d::Identity();
  }
  return true;
}

// ---------------------------------------------------------------- accelerometer
SpecificForceModel::SpecificForceModel(const Eigen::Isometry3d & mount, std::string bias_block, double gravity)
: mount_(mount), bias_block_(std::move(bias_block)), gravity_(gravity) {}

Eigen::VectorXd SpecificForceModel::predict(const State & x) const
{
  const Eigen::Vector3d r = mount_.translation();
  const Eigen::Vector3d w = x.angularVelocity();
  // Gravity pushes back on the sensor: at rest it reads +g upwards
  const Eigen::Vector3d up_in_body = x.orientation().conjugate() * Eigen::Vector3d(0.0, 0.0, gravity_);
  const Eigen::Vector3d f = x.linearAcceleration() + w.cross(w.cross(r)) + up_in_body;
  return mount_.rotation().transpose() * f + optionalBlock(x, bias_block_);
}

// ---------------------------------------------------------------- magnetometer
MagneticFieldModel::MagneticFieldModel(
  const Eigen::Quaterniond & mount_rotation, const Eigen::Vector3d & world_field, std::string offset_block)
: mount_rotation_(mount_rotation.normalized()), world_field_(world_field), offset_block_(std::move(offset_block)) {}

Eigen::VectorXd MagneticFieldModel::predict(const State & x) const
{
  return mount_rotation_.conjugate() * (x.orientation().conjugate() * world_field_) +
         optionalBlock(x, offset_block_);
}

// ---------------------------------------------------------------- subset
SubsetModel::SubsetModel(std::shared_ptr<const MeasurementModel> model, std::vector<int> components)
: model_(std::move(model)), components_(std::move(components))
{
  for (int c : components_) {
    if (c < 0 || c >= model_->dimension()) {
      throw std::invalid_argument("SubsetModel: component out of range");
    }
  }
}

Eigen::VectorXd SubsetModel::predict(const State & x) const
{
  const Eigen::VectorXd full = model_->predict(x);
  Eigen::VectorXd z(components_.size());
  for (std::size_t i = 0; i < components_.size(); ++i) {
    z(i) = full(components_[i]);
  }
  return z;
}

Eigen::VectorXd SubsetModel::residual(const Eigen::VectorXd & z, const Eigen::VectorXd & expected) const
{
  // Through the full model's residual (e.g. angle wrapping), the other components zero
  Eigen::VectorXd zf = Eigen::VectorXd::Zero(model_->dimension());
  Eigen::VectorXd ef = Eigen::VectorXd::Zero(model_->dimension());
  for (std::size_t i = 0; i < components_.size(); ++i) {
    zf(components_[i]) = z(i);
    ef(components_[i]) = expected(i);
  }
  const Eigen::VectorXd rf = model_->residual(zf, ef);
  Eigen::VectorXd r(components_.size());
  for (std::size_t i = 0; i < components_.size(); ++i) {
    r(i) = rf(components_[i]);
  }
  return r;
}

bool SubsetModel::jacobian(const State & x, Eigen::MatrixXd & H) const
{
  Eigen::MatrixXd full;
  if (!model_->jacobian(x, full)) {
    return false;
  }
  H.resize(components_.size(), full.cols());
  for (std::size_t i = 0; i < components_.size(); ++i) {
    H.row(i) = full.row(components_[i]);
  }
  return true;
}

// ---------------------------------------------------------------- stacked
StackedModel::StackedModel(std::vector<std::shared_ptr<const MeasurementModel>> models)
: models_(std::move(models)) {}

int StackedModel::dimension() const
{
  int n = 0;
  for (const auto & m : models_) {
    n += m->dimension();
  }
  return n;
}

Eigen::VectorXd StackedModel::predict(const State & x) const
{
  Eigen::VectorXd z(dimension());
  int row = 0;
  for (const auto & m : models_) {
    z.segment(row, m->dimension()) = m->predict(x);
    row += m->dimension();
  }
  return z;
}

Eigen::VectorXd StackedModel::residual(const Eigen::VectorXd & z, const Eigen::VectorXd & expected) const
{
  Eigen::VectorXd r(dimension());
  int row = 0;
  for (const auto & m : models_) {
    const int d = m->dimension();
    r.segment(row, d) = m->residual(z.segment(row, d), expected.segment(row, d));
    row += d;
  }
  return r;
}

bool StackedModel::jacobian(const State & x, Eigen::MatrixXd & H) const
{
  H.resize(dimension(), x.layout().tangentSize());
  int row = 0;
  for (const auto & m : models_) {
    Eigen::MatrixXd h;
    if (!m->jacobian(x, h)) {
      return false;
    }
    H.middleRows(row, m->dimension()) = h;
    row += m->dimension();
  }
  return true;
}

}  // namespace sac_localization

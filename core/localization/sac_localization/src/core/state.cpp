#include "sac_localization/core/state.hpp"

#include <stdexcept>

#include "sac_localization/core/so3.hpp"

namespace sac_localization
{

bool StateLayout::has(const std::string & block) const
{
  return index_.count(block) > 0;
}

const BlockInfo & StateLayout::block(const std::string & name) const
{
  auto it = index_.find(name);
  if (it == index_.end()) {
    throw std::out_of_range("No state block '" + name + "'");
  }
  return blocks_[it->second];
}

StateLayoutBuilder::StateLayoutBuilder()
{
  add(blocks::kPosition, BlockKind::kVector, 3);
  add(blocks::kOrientation, BlockKind::kRotation);
  add(blocks::kLinearVelocity, BlockKind::kVector, 3);
  add(blocks::kAngularVelocity, BlockKind::kVector, 3);
  add(blocks::kLinearAcceleration, BlockKind::kVector, 3);
}

void StateLayoutBuilder::add(const std::string & name, BlockKind kind, int size)
{
  if (kind == BlockKind::kRotation) {
    size = 3;
  }
  if (size <= 0) {
    throw std::invalid_argument("State block '" + name + "' needs a positive size");
  }
  for (const Entry & entry : entries_) {
    if (entry.name == name) {
      if (entry.kind != kind || entry.size != size) {
        throw std::invalid_argument("State block '" + name + "' added twice with different shapes");
      }
      return;
    }
  }
  entries_.push_back({name, kind, size});
}

std::shared_ptr<const StateLayout> StateLayoutBuilder::build() const
{
  auto layout = std::make_shared<StateLayout>();
  for (const Entry & entry : entries_) {
    BlockInfo info;
    info.name = entry.name;
    info.kind = entry.kind;
    info.tangent_size = entry.size;
    info.tangent_offset = layout->tangent_size_;
    if (entry.kind == BlockKind::kVector) {
      info.storage_index = layout->vector_storage_size_;
      layout->vector_storage_size_ += entry.size;
    } else {
      info.storage_index = layout->rotation_count_;
      layout->rotation_count_ += 1;
    }
    layout->tangent_size_ += entry.size;
    layout->index_[entry.name] = layout->blocks_.size();
    layout->blocks_.push_back(info);
  }
  return layout;
}

State::State(std::shared_ptr<const StateLayout> layout)
: layout_(std::move(layout)),
  values_(Eigen::VectorXd::Zero(layout_->vectorStorageSize())),
  rotations_(layout_->rotationCount(), Eigen::Quaterniond::Identity())
{
}

namespace
{
const BlockInfo & checked(const StateLayout & layout, const std::string & name, BlockKind kind)
{
  const BlockInfo & info = layout.block(name);
  if (info.kind != kind) {
    throw std::invalid_argument(
      "State block '" + name + "' is " + (kind == BlockKind::kVector ? "not a vector" : "not a rotation"));
  }
  return info;
}
}  // namespace

Eigen::Ref<Eigen::VectorXd> State::vector(const std::string & block)
{
  const BlockInfo & info = checked(*layout_, block, BlockKind::kVector);
  return values_.segment(info.storage_index, info.tangent_size);
}

Eigen::Ref<const Eigen::VectorXd> State::vector(const std::string & block) const
{
  const BlockInfo & info = checked(*layout_, block, BlockKind::kVector);
  return values_.segment(info.storage_index, info.tangent_size);
}

Eigen::Quaterniond & State::rotation(const std::string & block)
{
  return rotations_[checked(*layout_, block, BlockKind::kRotation).storage_index];
}

const Eigen::Quaterniond & State::rotation(const std::string & block) const
{
  return rotations_[checked(*layout_, block, BlockKind::kRotation).storage_index];
}

State State::boxplus(const Eigen::VectorXd & delta) const
{
  State result = *this;
  for (const BlockInfo & b : layout_->blocks()) {
    if (b.kind == BlockKind::kVector) {
      result.values_.segment(b.storage_index, b.tangent_size) += delta.segment(b.tangent_offset, b.tangent_size);
    } else {
      const Eigen::Vector3d d = delta.segment<3>(b.tangent_offset);
      result.rotations_[b.storage_index] = (rotations_[b.storage_index] * expSO3(d)).normalized();
    }
  }
  return result;
}

Eigen::VectorXd State::boxminus(const State & other) const
{
  Eigen::VectorXd delta(layout_->tangentSize());
  for (const BlockInfo & b : layout_->blocks()) {
    if (b.kind == BlockKind::kVector) {
      delta.segment(b.tangent_offset, b.tangent_size) =
        values_.segment(b.storage_index, b.tangent_size) -
        other.values_.segment(b.storage_index, b.tangent_size);
    } else {
      delta.segment<3>(b.tangent_offset) =
        logSO3(other.rotations_[b.storage_index].conjugate() * rotations_[b.storage_index]);
    }
  }
  return delta;
}

}  // namespace sac_localization

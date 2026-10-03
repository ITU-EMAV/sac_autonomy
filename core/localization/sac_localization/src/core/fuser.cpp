#include "sac_localization/core/fuser.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace sac_localization
{

Fuser::Fuser(
  std::shared_ptr<const StateLayout> layout, std::shared_ptr<Estimator> estimator,
  std::shared_ptr<MotionModel> motion_model, FuserOptions options)
: layout_(std::move(layout)),
  estimator_(std::move(estimator)),
  motion_model_(std::move(motion_model)),
  options_(options)
{
  if (!layout_ || !estimator_ || !motion_model_) {
    throw std::invalid_argument("Fuser needs a layout, an estimator and a motion model");
  }
}

void Fuser::reset(const Belief & initial)
{
  clear();
  history_.push_back(initial);
  current_ = initial;
  initialized_ = true;
}

void Fuser::clear()
{
  initialized_ = false;
  history_.clear();
  processed_.clear();
  pending_.clear();
  inputs_.clear();
  has_new_input_ = false;
  current_.reset();
}

void Fuser::widen(const std::string & block, const Eigen::VectorXd & variance)
{
  if (!initialized_) {
    return;
  }
  const BlockInfo & b = layout_->block(block);
  for (Belief * belief : {&history_.back(), &*current_}) {
    for (int i = 0; i < b.tangent_size && i < variance.size(); ++i) {
      belief->covariance(b.tangent_offset + i, b.tangent_offset + i) += variance(i);
    }
  }
}

void Fuser::add(Measurement measurement)
{
  if (!initialized_) {
    return;
  }
  pending_.push_back(std::move(measurement));
}

void Fuser::add(Input input)
{
  if (!initialized_) {
    return;
  }
  if (!has_new_input_ || input.stamp < earliest_new_input_) {
    earliest_new_input_ = input.stamp;
  }
  has_new_input_ = true;
  // Keep the inputs in stamp order; they nearly always arrive in order
  auto it = std::upper_bound(
    inputs_.begin(), inputs_.end(), input.stamp,
    [](Stamp stamp, const Input & other) { return stamp < other.stamp; });
  inputs_.insert(it, std::move(input));
}

Inputs Fuser::inputsAt(Stamp stamp) const
{
  Inputs inputs;
  for (const Input & input : inputs_) {
    if (input.stamp > stamp) {
      break;
    }
    inputs.set(input);  // later ones replace earlier ones of the same source
  }
  return inputs;
}

void Fuser::advance(Belief & belief, Stamp stamp) const
{
  const Stamp max_step = std::max<Stamp>(1, fromSeconds(options_.max_prediction_step));
  while (belief.stamp < stamp) {
    const Stamp end = std::min(stamp, belief.stamp + max_step);
    const double dt = toSeconds(end - belief.stamp);
    estimator_->predict(belief, dt, *motion_model_, inputsAt(belief.stamp));
    belief.stamp = end;
  }
}

Belief Fuser::predicted(Stamp stamp) const
{
  Belief belief = history_.back();
  advance(belief, stamp);
  return belief;
}

std::vector<UpdateResult> Fuser::update(Stamp now)
{
  std::vector<UpdateResult> results;
  if (!initialized_) {
    return results;
  }
  if (now < current_->stamp - fromSeconds(1.0)) {
    clear();  // the clock jumped back (e.g. the simulation restarted)
    return results;
  }

  // Ready: stamped up to now, in stamp order
  std::stable_sort(pending_.begin(), pending_.end(), [](const Measurement & a, const Measurement & b) {
    return a.stamp < b.stamp;
  });
  auto split = std::upper_bound(
    pending_.begin(), pending_.end(), now,
    [](Stamp stamp, const Measurement & m) { return stamp < m.stamp; });
  std::vector<Measurement> ready(
    std::make_move_iterator(pending_.begin()), std::make_move_iterator(split));
  pending_.erase(pending_.begin(), split);

  // Too late for the history: report and drop
  const Stamp oldest = history_.front().stamp;
  std::vector<Measurement> fresh;
  for (Measurement & m : ready) {
    if (m.stamp < oldest) {
      UpdateResult r;
      r.source = m.source;
      r.stamp = m.stamp;
      r.accepted = false;
      r.reason = "too late";
      results.push_back(r);
    } else {
      fresh.push_back(std::move(m));
    }
  }

  // Rewind to before the oldest new data, and replay what was fused after it
  Stamp earliest = fresh.empty() ? now : fresh.front().stamp;
  if (has_new_input_) {
    earliest = std::min(earliest, std::max(earliest_new_input_, oldest));
  }
  has_new_input_ = false;
  std::vector<std::pair<Measurement, bool>> queue;  // (measurement, is a replay)
  if (earliest < history_.back().stamp) {
    while (history_.size() > 1 && history_.back().stamp > earliest) {
      history_.pop_back();
    }
    const Stamp base = history_.back().stamp;
    while (!processed_.empty() && processed_.back().stamp > base) {
      queue.emplace_back(std::move(processed_.back()), true);
      processed_.pop_back();
    }
  }
  for (Measurement & m : fresh) {
    queue.emplace_back(std::move(m), false);
  }
  std::stable_sort(queue.begin(), queue.end(), [](const auto & a, const auto & b) {
    return a.first.stamp < b.first.stamp;
  });

  Belief belief = history_.back();
  for (auto & [m, replay] : queue) {
    advance(belief, m.stamp);
    UpdateResult r = estimator_->update(belief, m);
    r.stamp = m.stamp;
    r.source = m.source;
    history_.push_back(belief);
    processed_.push_back(std::move(m));
    if (!replay) {
      results.push_back(std::move(r));
    }
  }

  advance(belief, now);
  current_ = belief;
  prune(now);
  return results;
}

void Fuser::prune(Stamp now)
{
  const Stamp horizon = now - fromSeconds(options_.history);
  while (history_.size() > 1 && history_[1].stamp < horizon) {
    history_.pop_front();
  }
  const Stamp oldest = history_.front().stamp;
  while (!processed_.empty() && processed_.front().stamp <= oldest) {
    processed_.pop_front();
  }
  // Keep the last input before the oldest belief: it is still in effect after it
  while (inputs_.size() > 1 && inputs_[1].stamp <= oldest) {
    inputs_.pop_front();
  }
}

}  // namespace sac_localization

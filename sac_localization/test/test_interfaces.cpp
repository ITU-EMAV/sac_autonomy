// Design stage: checks that the interfaces compile and fit together (an estimator, a motion
// model and a measurement model can be written against them).

#include <gtest/gtest.h>

#include "sac_localization/core/estimator.hpp"
#include "sac_localization/core/fuser.hpp"
#include "sac_localization/core/geodesy.hpp"
#include "sac_localization/core/measurement.hpp"
#include "sac_localization/core/measurement_models.hpp"
#include "sac_localization/core/motion_model.hpp"
#include "sac_localization/core/params.hpp"
#include "sac_localization/core/state.hpp"
#include "sac_localization/ros/localization_node.hpp"
#include "sac_localization/ros/outputs.hpp"
#include "sac_localization/ros/sensor_adapter.hpp"

namespace sl = sac_localization;

class DummyEstimator : public sl::Estimator
{
public:
  void initialize(const sl::Params &) override {}
  void predict(sl::Belief &, double, const sl::MotionModel &, const sl::Inputs &) override {}
  sl::UpdateResult update(sl::Belief &, const sl::Measurement & m) override
  {
    sl::UpdateResult r;
    r.source = m.source;
    return r;
  }
};

class DummyMotionModel : public sl::MotionModel
{
public:
  void initialize(const sl::Params &) override {}
  sl::State predict(const sl::State & x, double, const sl::Inputs &) const override { return x; }
  Eigen::MatrixXd processNoise(const sl::State & x, double, const sl::Inputs &) const override
  {
    return Eigen::MatrixXd::Zero(x.layout().tangentSize(), x.layout().tangentSize());
  }
};

class DummyModel : public sl::MeasurementModel
{
public:
  int dimension() const override { return 1; }
  Eigen::VectorXd predict(const sl::State &) const override { return Eigen::VectorXd::Zero(1); }
};

TEST(Interfaces, FitTogether)
{
  std::unique_ptr<sl::Estimator> estimator = std::make_unique<DummyEstimator>();
  std::unique_ptr<sl::MotionModel> motion = std::make_unique<DummyMotionModel>();
  sl::Measurement m;
  m.source = "test";
  m.model = std::make_shared<DummyModel>();
  EXPECT_EQ(m.model->dimension(), 1);
  EXPECT_EQ(sl::fromSeconds(1.5), 1500000000);
  EXPECT_DOUBLE_EQ(sl::toSeconds(250000000), 0.25);
}

// The built-in estimators and motion models as pluginlib plugins (see plugins.xml).

#include <pluginlib/class_list_macros.hpp>

#include "sac_localization/estimators/ekf.hpp"
#include "sac_localization/estimators/ukf.hpp"
#include "sac_localization/motion_models/motion_models.hpp"

PLUGINLIB_EXPORT_CLASS(sac_localization::Ekf, sac_localization::Estimator)
PLUGINLIB_EXPORT_CLASS(sac_localization::Iekf, sac_localization::Estimator)
PLUGINLIB_EXPORT_CLASS(sac_localization::Ukf, sac_localization::Estimator)
PLUGINLIB_EXPORT_CLASS(sac_localization::ConstantAcceleration, sac_localization::MotionModel)
PLUGINLIB_EXPORT_CLASS(sac_localization::ImuDriven, sac_localization::MotionModel)
PLUGINLIB_EXPORT_CLASS(sac_localization::KinematicBicycle, sac_localization::MotionModel)

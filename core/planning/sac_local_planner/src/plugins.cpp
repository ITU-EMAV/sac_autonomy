// The built-in generator and cost functions as plugins (names in plugins.xml).

#include <pluginlib/class_list_macros.hpp>

#include "sac_local_planner/costs.hpp"
#include "sac_local_planner/frenet_lattice.hpp"

PLUGINLIB_EXPORT_CLASS(sac_local_planner::FrenetLattice, sac_local_planner::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(sac_local_planner::ObstacleClearance, sac_local_planner::CostFunction)
PLUGINLIB_EXPORT_CLASS(sac_local_planner::OffsetFromRoute, sac_local_planner::CostFunction)
PLUGINLIB_EXPORT_CLASS(sac_local_planner::LateralAcceleration, sac_local_planner::CostFunction)
PLUGINLIB_EXPORT_CLASS(sac_local_planner::Consistency, sac_local_planner::CostFunction)

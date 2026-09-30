// The built-in filters and sources as plugins (names in plugins.xml).

#include <pluginlib/class_list_macros.hpp>

#include "sac_perception/filters.hpp"
#include "sac_perception/ground_patchwork.hpp"
#include "sac_perception/map_representation.hpp"
#include "sac_perception/sources.hpp"

PLUGINLIB_EXPORT_CLASS(sac_perception::CropBox, sac_perception::PointFilter)
PLUGINLIB_EXPORT_CLASS(sac_perception::GroundHeight, sac_perception::PointFilter)
PLUGINLIB_EXPORT_CLASS(sac_perception::GroundPatchwork, sac_perception::PointFilter)
PLUGINLIB_EXPORT_CLASS(sac_perception::HeightBand, sac_perception::PointFilter)
PLUGINLIB_EXPORT_CLASS(sac_perception::Voxel, sac_perception::PointFilter)
PLUGINLIB_EXPORT_CLASS(sac_perception::PointCloudSource, sac_perception::GridSource)
PLUGINLIB_EXPORT_CLASS(sac_perception::LaserScanSource, sac_perception::GridSource)
PLUGINLIB_EXPORT_CLASS(sac_perception::OccupancyGridSource, sac_perception::GridSource)
PLUGINLIB_EXPORT_CLASS(sac_perception::DirectProjection, sac_perception::MapRepresentation)

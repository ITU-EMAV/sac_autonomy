cmake_minimum_required(VERSION 3.8)
project(sac_localization_adapters)

find_package(ament_cmake REQUIRED)

# Design stage: the adapters come after the core (see sac_localization/DESIGN.md).
# Vendor adapters (gps_msgs, geographic_msgs, ublox_msgs) will be built only when
# find_package() finds those packages, so the core never depends on them.

ament_package()

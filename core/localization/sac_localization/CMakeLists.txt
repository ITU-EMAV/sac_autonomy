cmake_minimum_required(VERSION 3.8)
project(sac_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sac_localization_msgs REQUIRED)

# Core: no ROS, Eigen only
add_library(sac_localization_core SHARED
  src/core/state.cpp
  src/core/geodesy.cpp
  src/core/measurement_models.cpp
  src/core/numeric.cpp
  src/core/fuser.cpp
  src/estimators/ekf.cpp
  src/estimators/ukf.cpp
  src/motion_models/motion_models.cpp
)
target_include_directories(sac_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sac_localization_core PUBLIC Eigen3::Eigen)

# The built-in estimators and motion models as plugins
add_library(sac_localization_plugins SHARED src/plugins.cpp)
target_link_libraries(sac_localization_plugins sac_localization_core pluginlib::pluginlib)
pluginlib_export_plugin_description_file(sac_localization plugins.xml)

# ROS: node, parameters, outputs
add_library(sac_localization_ros SHARED
  src/ros/ros_params.cpp
  src/ros/outputs.cpp
  src/ros/localization_node.cpp
)
target_link_libraries(sac_localization_ros PUBLIC
  sac_localization_core
  rclcpp::rclcpp
  rclcpp_components::component
  pluginlib::pluginlib
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sac_localization_msgs_TARGETS})
rclcpp_components_register_node(sac_localization_ros
  PLUGIN "sac_localization::LocalizationNode"
  EXECUTABLE localization_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS sac_localization_core sac_localization_plugins sac_localization_ros
  EXPORT export_sac_localization
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core
    test/test_state.cpp
    test/test_models.cpp
    test/test_estimators.cpp
    test/test_fuser.cpp
    test/test_motion_models.cpp)
  target_link_libraries(test_core sac_localization_core)
endif()

ament_export_targets(export_sac_localization HAS_LIBRARY_TARGET)
ament_export_dependencies(
  eigen3_cmake_module Eigen3 rclcpp rclcpp_components pluginlib tf2 tf2_ros tf2_eigen
  geometry_msgs nav_msgs sensor_msgs std_srvs sac_localization_msgs)
ament_package()

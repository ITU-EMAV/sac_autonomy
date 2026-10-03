cmake_minimum_required(VERSION 3.8)
project(sac_perception)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sac_perception_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Core: grid and filters, no ROS
add_library(sac_perception_core SHARED src/grid.cpp src/filters.cpp src/ground_patchwork.cpp
  src/map_representation.cpp src/column_map.cpp src/sparse_voxel.cpp
  src/multi_level_surface.cpp src/objects.cpp)
target_include_directories(sac_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sac_perception_core PUBLIC Eigen3::Eigen)

# ROS: sources and the node
add_library(sac_perception_ros SHARED src/sources.cpp src/perception_node.cpp src/robot_geometry.cpp)
target_link_libraries(sac_perception_ros PUBLIC
  sac_perception_core
  rclcpp::rclcpp
  rclcpp_components::component
  pluginlib::pluginlib
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${sac_perception_msgs_TARGETS}
  urdf::urdf
  ament_index_cpp::ament_index_cpp)
rclcpp_components_register_node(sac_perception_ros
  PLUGIN "sac_perception::PerceptionNode"
  EXECUTABLE perception_node)

# The built-in filters and sources as plugins
add_library(sac_perception_plugins SHARED src/plugins.cpp)
target_link_libraries(sac_perception_plugins sac_perception_ros)
pluginlib_export_plugin_description_file(sac_perception plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS sac_perception_core sac_perception_ros sac_perception_plugins
  EXPORT export_sac_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception test/test_grid.cpp test/test_filters.cpp test/test_benchmark.cpp
    test/test_sparse_voxel.cpp test/test_objects.cpp)
  target_link_libraries(test_perception sac_perception_core)
  ament_add_gtest(test_robot_geometry test/test_robot_geometry.cpp)
  target_link_libraries(test_robot_geometry sac_perception_ros)
endif()

ament_export_targets(export_sac_perception HAS_LIBRARY_TARGET)
ament_export_dependencies(
  eigen3_cmake_module Eigen3 rclcpp rclcpp_components pluginlib tf2 tf2_ros tf2_eigen
  sensor_msgs nav_msgs diagnostic_msgs std_msgs visualization_msgs sac_perception_msgs urdf ament_index_cpp)
ament_package()

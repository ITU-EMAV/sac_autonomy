cmake_minimum_required(VERSION 3.8)
project(sac_local_planner)

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
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sac_planning_msgs REQUIRED)
find_package(sac_perception_msgs REQUIRED)
find_package(sac_perception REQUIRED)

# Core: route, candidates, costs, collision checks and speeds, no ROS
add_library(sac_local_planner_core SHARED
  src/reference_path.cpp
  src/frenet_lattice.cpp
  src/costs.cpp
  src/local_planner.cpp)
target_include_directories(sac_local_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sac_local_planner_core PUBLIC Eigen3::Eigen sac_perception::sac_perception_core)

# The built-in generator and costs as plugins
add_library(sac_local_planner_plugins SHARED src/plugins.cpp)
target_link_libraries(sac_local_planner_plugins sac_local_planner_core pluginlib::pluginlib)
pluginlib_export_plugin_description_file(sac_local_planner plugins.xml)

# The node
add_library(sac_local_planner_ros SHARED src/planner_node.cpp)
target_link_libraries(sac_local_planner_ros PUBLIC
  sac_local_planner_core
  rclcpp::rclcpp
  rclcpp_components::component
  pluginlib::pluginlib
  tf2::tf2
  tf2_ros::tf2_ros
  sac_perception::sac_perception_core
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${sac_planning_msgs_TARGETS}
  ${sac_perception_msgs_TARGETS})
target_include_directories(sac_local_planner_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rclcpp_components_register_node(sac_local_planner_ros
  PLUGIN "sac_local_planner::LocalPlannerNode"
  EXECUTABLE local_planner_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS sac_local_planner_core sac_local_planner_plugins sac_local_planner_ros
  EXPORT export_sac_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_local_planner test/test_local_planner.cpp)
  target_link_libraries(test_local_planner sac_local_planner_core)
endif()

ament_export_targets(export_sac_local_planner HAS_LIBRARY_TARGET)
ament_export_dependencies(
  eigen3_cmake_module Eigen3 rclcpp rclcpp_components pluginlib tf2 tf2_ros nav_msgs
  visualization_msgs diagnostic_msgs sac_planning_msgs sac_perception)
ament_package()

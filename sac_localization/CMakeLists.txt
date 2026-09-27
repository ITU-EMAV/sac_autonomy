cmake_minimum_required(VERSION 3.8)
project(sac_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sac_localization_msgs REQUIRED)

# Design stage: the interfaces only. The core library (no ROS), the node, the estimators and
# the motion models come in the next steps (see DESIGN.md).
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Every header compiles on its own and together
  ament_add_gtest(test_interfaces test/test_interfaces.cpp)
  target_include_directories(test_interfaces PRIVATE include)
  target_link_libraries(test_interfaces
    Eigen3::Eigen rclcpp::rclcpp pluginlib::pluginlib tf2_ros::tf2_ros
    ${geometry_msgs_TARGETS} ${std_srvs_TARGETS})
endif()

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module Eigen3 rclcpp pluginlib tf2_ros tf2_eigen)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(sac_rviz_debug)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sac_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(${PROJECT_NAME}_panel SHARED
  src/perception_control_panel.cpp
  include/sac_rviz_debug/perception_control_panel.hpp)
target_include_directories(${PROJECT_NAME}_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_panel Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}_panel
  rviz_common pluginlib rclcpp sensor_msgs nav_msgs std_msgs sac_interfaces)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}_panel
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_package()

cmake_minimum_required(VERSION 3.8)
project(sac_pose_initializer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_component_interface_specs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sac_pose_initializer_node src/sac_pose_initializer_node.cpp)
ament_target_dependencies(sac_pose_initializer_node
  rclcpp geometry_msgs sensor_msgs autoware_component_interface_specs tf2_ros tf2_geometry_msgs tf2)

# NDT kilitlenmesini tespit edip otomatik yeniden hizalayan watchdog.
add_executable(ndt_recovery_watchdog_node src/ndt_recovery_watchdog_node.cpp)
ament_target_dependencies(ndt_recovery_watchdog_node
  rclcpp geometry_msgs diagnostic_msgs autoware_component_interface_specs)

install(TARGETS
  sac_pose_initializer_node
  ndt_recovery_watchdog_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

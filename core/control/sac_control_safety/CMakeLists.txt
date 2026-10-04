cmake_minimum_required(VERSION 3.8)
project(sac_control_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sac_interfaces REQUIRED)
find_package(cluster_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(command_guardian src/command_guardian.cpp)
ament_target_dependencies(command_guardian rclcpp std_srvs nav_msgs geometry_msgs
  sac_interfaces cluster_msgs)
target_compile_features(command_guardian PUBLIC cxx_std_17)
add_executable(odometry_pose_bridge src/odometry_pose_bridge.cpp)
ament_target_dependencies(odometry_pose_bridge rclcpp nav_msgs geometry_msgs)
target_compile_features(odometry_pose_bridge PUBLIC cxx_std_17)
add_executable(replay_localization_adapter src/replay_localization_adapter.cpp)
ament_target_dependencies(replay_localization_adapter rclcpp nav_msgs geometry_msgs
  sac_interfaces tf2 tf2_geometry_msgs)
target_compile_features(replay_localization_adapter PUBLIC cxx_std_17)

install(TARGETS command_guardian odometry_pose_bridge replay_localization_adapter
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/longitudinal_controller.py scripts/vehicle_speed_mux.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_launch_test(test/test_command_guardian.py TIMEOUT 45)
  add_launch_test(test/test_replay_localization_adapter.py TIMEOUT 45)
endif()

ament_package()

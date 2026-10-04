cmake_minimum_required(VERSION 3.8)
project(smart_car_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/lio_sam_auto_save.py
  scripts/saved_initial_pose.py
  scripts/ndt_ekf_lag_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  urdf
  rviz
  maps
  DESTINATION share/${PROJECT_NAME})
ament_package()

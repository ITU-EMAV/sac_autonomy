cmake_minimum_required(VERSION 3.8)
project(sac_network)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sac_interfaces REQUIRED)


add_executable(udp_sender src/udp_sender.cpp)
ament_target_dependencies(udp_sender rclcpp geometry_msgs sac_interfaces)
target_include_directories(udp_sender PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(udp_sender PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS udp_sender
  DESTINATION lib/${PROJECT_NAME})

add_executable(udp_receiver src/udp_receiver.cpp)
ament_target_dependencies(udp_receiver rclcpp sensor_msgs nav_msgs)
target_include_directories(udp_receiver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS udp_receiver DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

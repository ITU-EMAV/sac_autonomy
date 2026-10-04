cmake_minimum_required(VERSION 3.8)
project(pc_to_og_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(pc_to_og_node src/pc_to_og.cpp)
ament_target_dependencies(pc_to_og_node rclcpp sensor_msgs nav_msgs OpenCV)

install(TARGETS
  pc_to_og_node
  DESTINATION lib/${PROJECT_NAME})

include_directories(include)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES config/config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)


ament_package()

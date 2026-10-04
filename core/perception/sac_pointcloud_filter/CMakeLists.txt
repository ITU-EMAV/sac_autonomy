cmake_minimum_required(VERSION 3.8)
project(sac_pointcloud_filter)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)

add_executable(pointcloud_filter src/pointcloud_filter.cpp)
target_compile_features(pointcloud_filter PUBLIC cxx_std_17)
target_include_directories(pointcloud_filter SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pointcloud_filter rclcpp sensor_msgs pcl_conversions)
target_link_libraries(pointcloud_filter ${PCL_LIBRARIES})

install(TARGETS pointcloud_filter DESTINATION lib/${PROJECT_NAME})

ament_package()

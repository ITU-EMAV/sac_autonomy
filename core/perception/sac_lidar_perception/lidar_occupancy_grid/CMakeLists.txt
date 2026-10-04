cmake_minimum_required(VERSION 3.8)
project(lidar_occupancy_grid)

# Compile warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament and ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cluster_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# ** add these two lines **
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# include & link paths for PCL
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# executable: occupancy grid
add_executable(lidar_occupancy_grid src/occupancy_grid.cpp)
ament_target_dependencies(lidar_occupancy_grid
  rclcpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  cluster_msgs
  pcl_conversions
  OpenCV
)
# link in PCL libraries
target_link_libraries(lidar_occupancy_grid ${PCL_LIBRARIES})

# executable: lidar filter
add_executable(lidar_filter src/lidar_filter.cpp)
ament_target_dependencies(lidar_filter
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  pcl_conversions
)
target_link_libraries(lidar_filter ${PCL_LIBRARIES})

# install binaries
install(TARGETS
  lidar_occupancy_grid
  lidar_filter
  DESTINATION lib/${PROJECT_NAME}
)

# install launch and config
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()

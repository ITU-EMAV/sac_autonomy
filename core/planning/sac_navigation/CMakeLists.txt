cmake_minimum_required(VERSION 3.8)
project(sac_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)




include_directories(include) # include headers
FILE(GLOB_RECURSE HDR_FILES include/*.h*)
FILE(GLOB_RECURSE SRC_FILES src/*.c*)

add_executable(navigation ${HDR_FILES} ${SRC_FILES} src/navigation.cpp)

ament_target_dependencies(navigation rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

# Install our headers
install(
  DIRECTORY include/
  DESTINATION include
)


target_compile_features(navigation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS navigation
  DESTINATION lib/${PROJECT_NAME})

ament_package()

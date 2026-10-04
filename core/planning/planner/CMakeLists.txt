  cmake_minimum_required(VERSION 3.8)
  project(planner)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  include_directories(
    include
    include/${PROJECT_NAME}
    Thirdparty/AStar
    Thirdparty/Clipper2/CPP/Clipper2Lib/include
    Thirdparty/Spline
  )

  # Clipper2 source files
  file(GLOB CLIPPER2_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Clipper2/CPP/Clipper2Lib/src/*.cpp
  )

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sac_interfaces REQUIRED)
  find_package(cluster_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  # Lanelet2
  find_package(lanelet2_core REQUIRED)
  find_package(lanelet2_routing REQUIRED)
  find_package(lanelet2_projection REQUIRED)
  find_package(lanelet2_io REQUIRED)

  add_library(global_planner_container SHARED src/global_planner.cpp)
  add_library(mission_planner_container SHARED src/mission_planner.cpp)
  add_library(occupancy_grid_container SHARED
  src/occupancy_grid.cpp
  ${CLIPPER2_SRC}
  )

  
  add_library(trajectory_planner_container SHARED src/trajectory_planner.cpp)
  add_library(controller_container SHARED src/controller.cpp)
  add_library(reactive_road_planner_container SHARED src/reactive_road_planner.cpp)

  rclcpp_components_register_node(
      global_planner_container
      PLUGIN "smart_car::GlobalPlanner"
      EXECUTABLE global_planner_node
  )
  rclcpp_components_register_node(
      reactive_road_planner_container
      PLUGIN "smart_car::ReactiveRoadPlanner"
      EXECUTABLE reactive_road_planner_node
  )

  rclcpp_components_register_node(
      mission_planner_container
      PLUGIN "smart_car::MissionPlanner"
      EXECUTABLE mission_planner_node
  )

  rclcpp_components_register_node(
      occupancy_grid_container
      PLUGIN "smart_car::OccupancyGrid"
      EXECUTABLE occupancy_grid_node
  )

  rclcpp_components_register_node(
      trajectory_planner_container
      PLUGIN "smart_car::TrajectoryPlanner"
      EXECUTABLE trajectory_planner_node
  )

  rclcpp_components_register_node(
      controller_container
      PLUGIN "smart_car::Controller"
      EXECUTABLE controller_node
  )
  target_include_directories(mission_planner_container
  PUBLIC
    ${EIGEN3_INCLUDE_DIR}
  )

  ament_target_dependencies(global_planner_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs sac_interfaces cluster_msgs lanelet2_core lanelet2_routing lanelet2_projection lanelet2_io tf2_ros tf2_geometry_msgs visualization_msgs)
  ament_target_dependencies(mission_planner_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs sac_interfaces cluster_msgs lanelet2_core lanelet2_routing lanelet2_projection lanelet2_io tf2_geometry_msgs)
  ament_target_dependencies(occupancy_grid_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs sac_interfaces cluster_msgs tf2_ros cv_bridge tf2_geometry_msgs lanelet2_core lanelet2_projection lanelet2_io visualization_msgs) 
  ament_target_dependencies(trajectory_planner_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs sac_interfaces lanelet2_core lanelet2_projection lanelet2_io tf2_ros tf2_geometry_msgs visualization_msgs)
  ament_target_dependencies(controller_container rclcpp rclcpp_components nav_msgs geometry_msgs std_msgs sac_interfaces tf2_ros tf2_geometry_msgs tf2 visualization_msgs)
  ament_target_dependencies(reactive_road_planner_container rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs cv_bridge OpenCV)

  install(TARGETS 
      global_planner_container 
      mission_planner_container
      occupancy_grid_container
      trajectory_planner_container 
      controller_container
      reactive_road_planner_container
    EXPORT export_${PROJECT_NAME}_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(PROGRAMS scripts/traffic_speed_planner.py scripts/road_hazard_speed_planner.py
    scripts/map_speed_constraint.py scripts/behavior_manager.py DESTINATION lib/${PROJECT_NAME})

  # Install launch files
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  # Install config files
  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )

  ament_export_targets(export_${PROJECT_NAME}_targets)

  ament_package()

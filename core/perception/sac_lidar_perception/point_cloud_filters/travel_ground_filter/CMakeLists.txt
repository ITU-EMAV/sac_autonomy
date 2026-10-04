cmake_minimum_required(VERSION 3.8)
project(ground_segmentation)

# Set CMake policy for handling _ROOT variables
cmake_policy(SET CMP0074 NEW)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io visualization filters)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions  REQUIRED)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Find OpenMP for parallelization
find_package(OpenMP REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
        include
        SYSTEM
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}        
        ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(ground_segmentation SHARED
        src/travel_ground_filter_nodelet.cpp
)

if(${PCL_VERSION} GREATER_EQUAL 1.12.1)
    find_package(Qhull REQUIRED)
    target_link_libraries(ground_segmentation
            QHULL::QHULL
            )
endif()

target_link_libraries(ground_segmentation
        Eigen3::Eigen
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        OpenMP::OpenMP_CXX
)

# Install config files
install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

rclcpp_components_register_node(ground_segmentation
        PLUGIN "ground_segmentation::TravelGroundFilterComponent"
        EXECUTABLE travel_ground_filter_node

)

ament_auto_package()

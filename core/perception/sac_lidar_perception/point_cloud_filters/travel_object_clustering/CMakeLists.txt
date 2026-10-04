cmake_minimum_required(VERSION 3.8)
project(object_clustering)

# Set CMake policy for handling _ROOT variables
cmake_policy(SET CMP0074 NEW)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io surface features)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(FLANN REQUIRED) # Ensure FLANN is found
find_package(cluster_msgs REQUIRED)

# Ensure all dependencies are found
ament_auto_find_build_dependencies()

# Include directories
include_directories(
        include
        SYSTEM
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${FLANN_INCLUDE_DIRS} # Include FLANN directories
        ${cluster_msgs_INCLUDE_DIRS}
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add library
ament_auto_add_library(object_clustering SHARED
        src/travel_object_clustering_nodelet.cpp
)

# Check for PCL version and link Qhull if necessary
if(${PCL_VERSION} GREATER_EQUAL 1.12.1)
    find_package(Qhull REQUIRED)
    target_link_libraries(object_clustering
            QHULL::QHULL
    )
endif()

# Link necessary libraries
target_link_libraries(object_clustering
        Eigen3::Eigen
        ${Boost_LIBRARIES}
        ${PCL_LIBRARIES}
        ${FLANN_LIBRARIES} # Link FLANN libraries
        ${cluster_msgs_LIBRARIES}
)

# Install config files
install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# Register node
rclcpp_components_register_node(object_clustering
        PLUGIN "object_clustering::TravelObjectClusteringComponent"
        EXECUTABLE travel_object_clustering_node
)

# Finalize package
ament_auto_package()

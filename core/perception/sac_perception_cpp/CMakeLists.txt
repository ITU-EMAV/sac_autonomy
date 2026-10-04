cmake_minimum_required(VERSION 3.8)
project(sac_perception_cpp)


# add_compile_options(-Wall -Wextra -Wpedantic -Wno-write-strings)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()


set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/usr/local/lib/python3.10/dist-packages/torch/share/cmake")



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Torch REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(ZED 4 REQUIRED)
find_package(CUDA REQUIRED)


include_directories(include ${cv_bridge_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIR})


link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})



IF(NOT WIN32)
    SET(SPECIAL_OS_LIBS "pthread")
ENDIF()

FILE(GLOB_RECURSE SRC_FILES src/*.c*)
FILE(GLOB_RECURSE HDR_FILES include/*.h*)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})

function(custom_executable target)
  add_executable(${target} ${HDR_FILES} ${SRC_FILES} src/${target}.cpp)
  ament_target_dependencies(${target}
  "example_interfaces"
  "rclcpp"
  "sensor_msgs"
  "visualization_msgs"
  "geometry_msgs"
  "nav_msgs"
  "rcutils"
  "std_msgs"
  "cv_bridge"
  "OpenCV"
  )
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
  
endfunction()


if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()



custom_executable(perception_pcl)
target_link_libraries(perception_pcl ${TORCH_LIBRARIES})
target_link_libraries(perception_pcl ${ZED_LIBS} ${SPECIAL_OS_LIBS} ${GLUT_LIBRARIES} ${GLEW_LIBRARIES})



ament_package()
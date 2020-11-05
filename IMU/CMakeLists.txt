cmake_minimum_required(VERSION 2.8.3)
project(gy85-raspberrypi)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#find_package(Eigen3 REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_library(gy85 SHARED
    adxl345/adxl345.c
    hmc5883l/hmc5883l.c
    itg3200/itg3200.c
)


add_executable(gy85_test demo.cc)
target_link_libraries(gy85_test 
  gy85
  -lncurses
) 

add_executable(ros_pub Ros/ros_pub.c)
target_link_libraries(ros_pub 
  gy85
  -lncurses
) 



cmake_minimum_required(VERSION 2.8.3)
project(lol)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cloud_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  alego
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES lol_localization
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(lol_localization src/lolLocalization.cpp)
target_link_libraries(lol_localization 
  ${CERES_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(lol src/main.cpp)
target_link_libraries(lol lol_localization)

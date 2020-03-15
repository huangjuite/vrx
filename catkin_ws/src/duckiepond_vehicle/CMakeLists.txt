cmake_minimum_required(VERSION 2.8.3)
project(duckiepond_vehicle)


find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)



## Gazebo
find_package(gazebo REQUIRED)

## Eigen
find_package(Eigen3 REQUIRED)

if (NOT MSVC)
  set(UNFILTERED_FLAGS "-std=c++11")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11 -Wall -Wextra")

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  UsvDrive.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp message_runtime std_msgs geometry_msgs nav_msgs sensor_msgs
)

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


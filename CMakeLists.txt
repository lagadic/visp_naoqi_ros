cmake_minimum_required(VERSION 2.8.3)
project(visp_naoqi_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(visp_naoqi REQUIRED)


catkin_package(
  
#  INCLUDE_DIRS
#  include
  
  LIBRARIES
  visp_naoqi_robot
  visp_naoqi_ros
  
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf 

  DEPENDS 
  visp_naoqi
)

#include_directories(include)

include_directories(
  ${VISP_NAOQI_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(visp_naoqi_robot
  src/visp_naoqi_robot.cpp
)
target_link_libraries(visp_naoqi_robot
  ${catkin_LIBRARIES}
  ${VISP_NAOQI_LIBRARIES}
)

## Declare a cpp executable
add_executable(visp_naoqi_ros src/main.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(visp_naoqi_ros
  ${catkin_LIBRARIES}
  ${VISP_NAOQI_LIBRARIES} 
  visp_naoqi_robot
)

cmake_minimum_required(VERSION 2.8.3)
project(zed_custom_sub_tutorial)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs zed_interfaces geometry_msgs std_msgs GeographicLib message_generation message_runtime ublox_serialization)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

## Build 
add_executable(zed_custom_sub src/zed_custom_sub_tutorial.cpp)
target_link_libraries(zed_custom_sub ${catkin_LIBRARIES})
add_dependencies(zed_custom_sub ${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 2.8.3)
project(skye_controls)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  skye_ros
  eigen_conversions
)


catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${skye_ros_INCLUDE_DIRS}
  include
)

## Declare a C++ executable
add_executable(skye_controls_node
                    src/main.cpp
                    src/skye_position_controller.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(skye_controls_node
                          ${catkin_LIBRARIES}
                          ${roslib_LIBRARIES}
                          ${skye_ros_LIBRARIES}
 )

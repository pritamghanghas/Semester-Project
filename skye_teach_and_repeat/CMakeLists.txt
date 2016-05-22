cmake_minimum_required(VERSION 2.8.3)
project(skye_teach_and_repeat)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  skye_ros
  eigen_conversions
std_msgs
skye_controls
)

# make sure configure headers are built before any node using them
#add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${skye_ros_INCLUDE_DIRS}
  ${skye_controls_INCLUDE_DIRS}
  include
)


## Declare a C++ executable
add_executable(skye_teach_and_repeat_node
                    src/skye_teach_and_repeat_node.cpp)

#add_library(skye_gazebo_blimp_dynamics_plugin src/gazebo_blimp_dynamics_plugin.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(skye_teach_and_repeat_node
                          ${catkin_LIBRARIES}
                          ${roslib_LIBRARIES}
                          ${skye_ros_LIBRARIES}
                          ${skye_controls_LIBRARIES}
 )
target_link_libraries(skye_teach_and_repeat_node waypoint_ctrl)


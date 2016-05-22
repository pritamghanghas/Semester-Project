cmake_minimum_required(VERSION 2.8.3)
project(skye_controls)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  skye_ros
  cmake_modules
  eigen_conversions
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(
  cfg/skye_params.cfg
)
# make sure configure headers are built before any node using them
#add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${skye_ros_INCLUDE_DIRS}
  include
  src/node
)

## Declare a C++ executable
add_executable(skye_controls_node
                    src/node/pose_control_node.cpp
                    src/node/waypoints_parser.cpp
		    src/library/waypoint_controller.cpp 
		    src/library/skye_geometric_controller.cpp)

target_compile_features(skye_controls_node PRIVATE cxx_range_for)

## Specify libraries to link a library or executable target against
target_link_libraries(skye_controls_node
                          ${catkin_LIBRARIES}
                          ${roslib_LIBRARIES}
                          ${skye_ros_LIBRARIES}
 )



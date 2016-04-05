cmake_minimum_required(VERSION 2.8.3)
project(skye_tests)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  cmake_modules
  tf_conversions
  eigen_conversions
  message_generation
  skye_ros
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
add_executable(skye_tests_node
                    src/main.cpp
                    src/action_skye.cpp
                    src/skye.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(skye_tests_node
                          ${catkin_LIBRARIES}
                          ${roslib_LIBRARIES}
                          ${skye_ros_LIBRARIES}
 )

cmake_minimum_required(VERSION 3.0.2)
project(ros-dynamic-reconfigure)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/RosDynamicReconfigure.cfg
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_node src/dynamic_node.cpp src/dynamic.cpp)
add_dependencies(dynamic_node ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamic_node ${catkin_LIBRARIES})
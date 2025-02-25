cmake_minimum_required(VERSION 3.0.2)
project(drone_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs gazebo_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(drone_control_node src/drone_control_node.cpp)
target_link_libraries(drone_control_node ${catkin_LIBRARIES})
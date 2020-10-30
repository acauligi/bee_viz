cmake_minimum_required(VERSION 2.8.3)
project(bee_viz)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  visualization_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  message_filters
  tf
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bee_viz 
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(render src/render.cpp)
target_link_libraries(render ${catkin_LIBRARIES})

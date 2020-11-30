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
find_package(Bullet REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(scp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bee_viz 
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

message(STATUS "scp dirs: ${scp_INCLUDE_DIRS}")
include_directories(SYSTEM
  ${scp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${BULLET_INCLUDE_DIRS}
)

add_executable(render src/render.cpp)
target_link_libraries(render
  PUBLIC
  ${catkin_LIBRARIES}
  PRIVATE
  scp
  osqp::osqp
)

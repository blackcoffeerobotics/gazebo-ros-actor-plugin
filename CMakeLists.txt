cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_actor_plugin)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  geometry_msgs
  nav_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    gazebo_ros_actor_plugin
  CATKIN_DEPENDS
    gazebo_ros
    gazebo_plugins
    geometry_msgs
    nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gazebo_ros_actor src/gazebo_ros_actor.cpp)
target_link_libraries(gazebo_ros_actor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_path_follow_actor src/gazebo_ros_path_follow_actor.cpp)
target_link_libraries(gazebo_ros_path_follow_actor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_robot_follow_actor src/gazebo_ros_robot_follow_actor.cpp)
target_link_libraries(gazebo_ros_robot_follow_actor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(gazebo_ros_temporary_stop_actor src/gazebo_ros_temporary_stop_actor.cpp)
target_link_libraries(gazebo_ros_temporary_stop_actor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

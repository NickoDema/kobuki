cmake_minimum_required(VERSION 2.8.3)
project(agv_navigation)

find_package(catkin REQUIRED COMPONENTS
  move_base
  gmapping
  move_base_msgs
)

catkin_python_setup()


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agv_navigation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

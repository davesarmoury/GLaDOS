cmake_minimum_required(VERSION 3.0.2)
project(glados_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES glados_description
#  CATKIN_DEPENDS zed_interfaces zed_wrapper
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

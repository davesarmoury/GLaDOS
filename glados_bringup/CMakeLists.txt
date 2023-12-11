cmake_minimum_required(VERSION 3.0.2)
project(glados_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES glados_bringup
#  CATKIN_DEPENDS glados_description glados_riva openai_ros zed_wrapper
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

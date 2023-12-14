cmake_minimum_required(VERSION 3.0.2)
project(glados_control)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES glados_control
#  CATKIN_DEPENDS tf trac_ik_kinematics_plugin zed_interfaces
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

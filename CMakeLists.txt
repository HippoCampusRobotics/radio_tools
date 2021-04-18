cmake_minimum_required(VERSION 3.0.2)
project(radio_tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hippocampus_common
  rospy
  rqt_gui
)

catkin_python_setup()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES radio_tools
#  CATKIN_DEPENDS geometry_msgs hippocampus_common pyserial rospy rqt
#  DEPENDS system_lib
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


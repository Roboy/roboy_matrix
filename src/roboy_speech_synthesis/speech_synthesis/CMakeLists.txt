cmake_minimum_required(VERSION 2.8.3)
project(speech_synthesis)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roboy_communication_cognition
  std_msgs
)


catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES speech_synthesis
  CATKIN_DEPENDS  rospy roboy_communication_cognition std_msgs
#  DEPENDS system_lib
)


#############
## Install ##
#############
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   scripts/tts_server.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



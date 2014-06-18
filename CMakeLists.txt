cmake_minimum_required(VERSION 2.8.3)
project(mavros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  message_generation
  message_runtime
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system thread date_time)
find_package(mavlink REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Mavlink.msg
  Waypoint.msg
  WaypointList.msg
  BatteryStatus.msg
  State.msg
  RCIn.msg
  RCOut.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ParamSet.srv
   ParamGet.srv
   ParamPull.srv
   ParamPush.srv
   WaypointSetCurrent.srv
   WaypointClear.srv
   WaypointPull.srv
   WaypointPush.srv
   WaypointGOTO.srv
   OverrideRCIn.srv
   CommandLong.srv
   CommandBool.srv
   CommandMode.srv
   CommandHome.srv
   StreamRate.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  diagnostic_msgs
  sensor_msgs
  std_msgs
)

## Select MAVLink dialect
#
# TODO: move dialect list to mavlink package

set(MAVLINK_DIALECT "ardupilotmega" CACHE STRING "MAVLink dialect selector")
set_property(CACHE MAVLINK_DIALECT PROPERTY STRINGS
  "ardupilotmega"
  "autoquad"
  "common"
  "matrixpilot"
  "minimal"
  "pixhawk"
  "slugs"
  "test"
  "ualberta"
  "sensesoar"
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavconn mavros_plugins
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater pluginlib roscpp sensor_msgs std_msgs tf geometry_msgs
  DEPENDS Boost mavlink
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# TODO: Check names of system library include directories (boost)
include_directories(
  include
  src/mavconn
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
)

add_definitions(
    -DMAVLINK_DIALECT=${MAVLINK_DIALECT}
)

## Enable C++11 on gcc 4.6 too
# thanks for: http://www.guyrutenberg.com/2014/01/05/enabling-c11-c0x-in-cmake/
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Declare a cpp library
add_library(mavconn
   src/mavconn/mavconn_interface.cpp
   src/mavconn/mavconn_serial.cpp
   src/mavconn/mavconn_udp.cpp
)
add_dependencies(mavconn ${MAVLINK_DIALECT}.xml-v1.0)
target_link_libraries(mavconn
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(mavros_plugins
  src/plugins/dummy.cpp
  src/plugins/sys_status.cpp
  src/plugins/imu_pub.cpp
  src/plugins/gps.cpp
  src/plugins/param.cpp
  src/plugins/waypoint.cpp
  src/plugins/rc_io.cpp
  src/plugins/command.cpp
)
add_dependencies(mavros_plugins
  mavros_generate_messages_cpp
  mavconn
)
target_link_libraries(mavros_plugins
  mavconn
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


## Declare a cpp executable
add_executable(mavros_node
	src/mavros_node.cpp
	src/uas.cpp
)
add_dependencies(mavros_node
  mavros_generate_messages_cpp
  mavconn
)
target_link_libraries(mavros_node
  mavconn
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(ros_udp
	src/ros_udp.cpp
)
add_dependencies(ros_udp
  mavconn
)
target_link_libraries(ros_udp
  mavconn
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/mavparam
  scripts/mavwp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS mavconn mavros_plugins mavros_node ros_udp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  mavros_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(mavconn-test test/test_mavconn.cpp)
if(TARGET mavconn-test)
	target_link_libraries(mavconn-test mavconn)
endif()

catkin_add_gtest(mavudpproxy test/mavudpproxy.cpp)
if(TARGET mavudpproxy)
	target_link_libraries(mavudpproxy mavconn)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

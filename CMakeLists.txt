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
find_package(Boost REQUIRED COMPONENTS system thread timer date_time)


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
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ParamSet.srv
   ParamGet.srv
   ParamPull.srv
   ParamPush.srv
   WaypointSet.srv
   WaypointPull.srv
   WaypointPush.srv
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

## Generate MAVLink headers
include(${CMAKE_CURRENT_SOURCE_DIR}/mavlink.cmake)

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

set(MAVLINK_DEF
  ardupilotmega.xml
  autoquad.xml
  common.xml
  matrixpilot.xml
  minimal.xml
  pixhawk.xml
  slugs.xml
  test.xml
  ualberta.xml
  sensesoar.xml
)

generate_mavlink("1.0" "${MAVLINK_DEF}")


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
  LIBRARIES mavconn
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater pluginlib roscpp sensor_msgs std_msgs tf geometry_msgs
  DEPENDS boost
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
  ${boost_INCLUDE_DIRS}
)

add_definitions(
    -std=c++11
    -DMAVLINK_DIALECT=${MAVLINK_DIALECT}
)

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
  ${boost_LIBRARIES}
)

add_library(mavros_plugins
  src/plugins/dummy.cpp
  src/plugins/sys_status.cpp
  src/plugins/imu_pub.cpp
  src/plugins/gps.cpp
  src/plugins/param.cpp
)
add_dependencies(mavros_plugins
  mavros_generate_messages_cpp
  mavconn
)
target_link_libraries(mavros_plugins
  mavconn
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${boost_LIBRARIES}
)


## Declare a cpp executable
add_executable(mavros_node src/mavros_node.cpp)
add_dependencies(mavros_node
  mavros_generate_messages_cpp
  mavconn
)
target_link_libraries(mavros_node
  mavconn
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${boost_LIBRARIES}
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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS mavconn mavros_plugins mavros_node
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
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

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

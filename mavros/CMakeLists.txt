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
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  angles
  libmavconn
  rosconsole_bridge
  eigen_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

include(EnableCXX11)
include(MavrosMavlink)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

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
  RadioStatus.msg
  OverrideRCIn.msg
  VFR_HUD.msg
  FileEntry.msg
  ActuatorControl.msg
  CommandCode.msg
  ParamValue.msg
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
  CommandLong.srv
  CommandInt.srv
  CommandBool.srv
  CommandHome.srv
  CommandTOL.srv
  CommandTriggerControl.srv
  StreamRate.srv
  SetMode.srv
  FileOpen.srv
  FileClose.srv
  FileRead.srv
  FileWrite.srv
  FileList.srv
  FileRemove.srv
  FileMakeDir.srv
  FileRemoveDir.srv
  FileTruncate.srv
  FileRename.srv
  FileChecksum.srv
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
  LIBRARIES mavros
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater pluginlib roscpp sensor_msgs std_msgs tf2_ros geometry_msgs libmavconn message_runtime eigen_conversions
  DEPENDS Boost Eigen
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
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mavros
  src/lib/uas_data.cpp
  src/lib/uas_stringify.cpp
  src/lib/uas_timesync.cpp
  src/lib/uas_sensor_orientation.cpp
  src/lib/uas_frame_conversions.cpp
  src/lib/mavros.cpp
  src/lib/mavlink_diag.cpp
  src/lib/rosconsole_bridge.cpp
)
add_dependencies(mavros
  mavros_generate_messages_cpp
)
target_link_libraries(mavros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(mavros_plugins
  src/plugins/dummy.cpp
  src/plugins/sys_status.cpp
  src/plugins/sys_time.cpp
  src/plugins/imu_pub.cpp
  src/plugins/param.cpp
  src/plugins/waypoint.cpp
  src/plugins/rc_io.cpp
  src/plugins/command.cpp
  src/plugins/local_position.cpp
  src/plugins/global_position.cpp
  src/plugins/setpoint_position.cpp
  src/plugins/safety_area.cpp
  src/plugins/3dr_radio.cpp
  src/plugins/setpoint_velocity.cpp
  src/plugins/setpoint_accel.cpp
  src/plugins/setpoint_attitude.cpp
  src/plugins/vfr_hud.cpp
  src/plugins/ftp.cpp
  src/plugins/actuator_control.cpp
)
add_dependencies(mavros_plugins
  mavros
)
target_link_libraries(mavros_plugins
  mavros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Declare a cpp executable
add_executable(mavros_node
  src/mavros_node.cpp
)
add_dependencies(mavros_node
  mavros
)
target_link_libraries(mavros_node
  mavros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(gcs_bridge
  src/gcs_bridge.cpp
)
add_dependencies(gcs_bridge
  mavros_generate_messages_cpp
)
target_link_libraries(gcs_bridge
  mavros
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
file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*)
catkin_install_python(PROGRAMS
  ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS mavros mavros_plugins mavros_node gcs_bridge
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

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

catkin_add_gtest(libmavros-frame-test test/test_frame_conv.cpp)
target_link_libraries(libmavros-frame-test mavros)

catkin_add_gtest(libmavros-sens-orientation-test test/test_sensor_orientation.cpp)
target_link_libraries(libmavros-sens-orientation-test mavros)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

endif()

# vim: ts=2 sw=2 et:

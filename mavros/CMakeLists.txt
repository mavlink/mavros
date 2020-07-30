cmake_minimum_required(VERSION 2.8.3)
project(mavros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  diagnostic_msgs
  diagnostic_updater
  eigen_conversions
  geographic_msgs
  geometry_msgs
  libmavconn
  mavros_msgs
  nav_msgs
  pluginlib
  rosconsole_bridge
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  tf2_ros
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Find Eigen
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

## Find GeographicLib
find_package(GeographicLib REQUIRED)

## Check if the datasets are installed
include(CheckGeographicLibDatasets)

include(EnableCXX11)
include(MavrosMavlink)

# detect if sensor_msgs has BatteryState.msg
# http://answers.ros.org/question/223769/how-to-check-that-message-exists-with-catkin-for-conditional-compilation-sensor_msgsbatterystate/
list(FIND sensor_msgs_MESSAGE_FILES "msg/BatteryState.msg" BATTERY_STATE_MSG_IDX)
if(${BATTERY_STATE_MSG_IDX} GREATER -1)
  add_definitions(
    -DHAVE_SENSOR_MSGS_BATTERYSTATE_MSG
  )
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


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
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater eigen_conversions geographic_msgs geometry_msgs libmavconn mavros_msgs message_runtime nav_msgs pluginlib roscpp sensor_msgs std_msgs tf2_ros trajectory_msgs
  DEPENDS Boost EIGEN3 GeographicLib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(mavros
  src/lib/enum_sensor_orientation.cpp
  src/lib/enum_to_string.cpp
  src/lib/ftf_frame_conversions.cpp
  src/lib/ftf_quaternion_utils.cpp
  src/lib/mavlink_diag.cpp
  src/lib/mavros.cpp
  src/lib/rosconsole_bridge.cpp
  src/lib/uas_data.cpp
  src/lib/uas_stringify.cpp
  src/lib/uas_timesync.cpp
)
add_dependencies(mavros
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mavros
  atomic
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_library(mavros_plugins
  src/plugins/3dr_radio.cpp
  src/plugins/actuator_control.cpp
  src/plugins/altitude.cpp
  src/plugins/command.cpp
  src/plugins/dummy.cpp
  src/plugins/ftp.cpp
  src/plugins/global_position.cpp
  src/plugins/hil.cpp
  src/plugins/home_position.cpp
  src/plugins/imu.cpp
  src/plugins/local_position.cpp
  src/plugins/manual_control.cpp
  src/plugins/param.cpp
  src/plugins/rc_io.cpp
  src/plugins/safety_area.cpp
  src/plugins/setpoint_accel.cpp
  src/plugins/setpoint_attitude.cpp
  src/plugins/setpoint_position.cpp
  src/plugins/setpoint_raw.cpp
  src/plugins/setpoint_velocity.cpp
  src/plugins/setpoint_trajectory.cpp
  src/plugins/sys_status.cpp
  src/plugins/sys_time.cpp
  src/plugins/vfr_hud.cpp
  src/plugins/waypoint.cpp
  src/plugins/wind_estimation.cpp
)
add_dependencies(mavros_plugins
  mavros
)
target_link_libraries(mavros_plugins
  mavros
  ${catkin_LIBRARIES}
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
)

add_executable(gcs_bridge
  src/gcs_bridge.cpp
)
target_link_libraries(gcs_bridge
  mavros
  ${catkin_LIBRARIES}
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
install(TARGETS gcs_bridge mavros mavros_node mavros_plugins
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
  catkin_add_gtest(libmavros-frame-conversions-test test/test_frame_conversions.cpp)
  target_link_libraries(libmavros-frame-conversions-test mavros)

  catkin_add_gtest(libmavros-sensor-orientation-test test/test_sensor_orientation.cpp)
  target_link_libraries(libmavros-sensor-orientation-test mavros)

  catkin_add_gtest(libmavros-quaternion-utils-test test/test_quaternion_utils.cpp)
  target_link_libraries(libmavros-quaternion-utils-test mavros)
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

endif()

# vim: ts=2 sw=2 et:

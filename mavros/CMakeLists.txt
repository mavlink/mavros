cmake_minimum_required(VERSION 3.5)
project(mavros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)

# find mavros dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)

find_package(mavlink REQUIRED)
find_package(libmavconn REQUIRED)
find_package(console_bridge REQUIRED)
#find_package(rosconsole_bridge REQUIRED)  # XXX TODO: connect libmavconn loggers

find_package(Eigen3 REQUIRED)
find_package(GEOGRAPHICLIB REQUIRED)

find_package(angles REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

#find_package(diagnostic_msgs REQUIRED)
#find_package(diagnostic_updater REQUIRED)

find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

include_directories(
  ${mavlink_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GEOGRAPHICLIB_INCLUDE_DIRS}
)

## Check if the datasets are installed
include(CheckGeographicLibDatasets)

# detect if sensor_msgs has BatteryState.msg
# http://answers.ros.org/question/223769/how-to-check-that-message-exists-with-catkin-for-conditional-compilation-sensor_msgsbatterystate/
#list(FIND sensor_msgs_MESSAGE_FILES "msg/BatteryState.msg" BATTERY_STATE_MSG_IDX)
#if(${BATTERY_STATE_MSG_IDX} GREATER -1)
#  add_definitions(
#    -DHAVE_SENSOR_MSGS_BATTERYSTATE_MSG
#    )
#endif()

add_library(mavros SHARED
  src/lib/enum_sensor_orientation.cpp
  src/lib/enum_to_string.cpp
  src/lib/ftf_frame_conversions.cpp
  src/lib/ftf_quaternion_utils.cpp
  src/lib/mavros_router.cpp
#  src/lib/mavlink_diag.cpp
#  src/lib/mavros.cpp
#  src/lib/rosconsole_bridge.cpp
#  src/lib/uas_data.cpp
#  src/lib/uas_stringify.cpp
#  src/lib/uas_timesync.cpp
)
ament_target_dependencies(mavros
  rclcpp
  rclcpp_components
  rcpputils
  #class_loader
  #sensor_msgs
  pluginlib
  mavros_msgs
  libmavconn
  console_bridge
)
target_link_libraries(mavros ${GEOGRAPHICLIB_LIBRARIES})
rclcpp_components_register_nodes(mavros "mavros::router::Router")
#rclcpp_components_register_nodes(mavros "MavrosVehicle")

#add_library(mavros_plugins
#  src/plugins/3dr_radio.cpp
#  src/plugins/actuator_control.cpp
#  src/plugins/altitude.cpp
#  src/plugins/command.cpp
#  src/plugins/dummy.cpp
#  src/plugins/ftp.cpp
#  src/plugins/global_position.cpp
#  src/plugins/hil.cpp
#  src/plugins/home_position.cpp
#  src/plugins/imu.cpp
#  src/plugins/local_position.cpp
#  src/plugins/manual_control.cpp
#  src/plugins/param.cpp
#  src/plugins/rc_io.cpp
#  src/plugins/safety_area.cpp
#  src/plugins/setpoint_accel.cpp
#  src/plugins/setpoint_attitude.cpp
#  src/plugins/setpoint_position.cpp
#  src/plugins/setpoint_raw.cpp
#  src/plugins/setpoint_velocity.cpp
#  src/plugins/setpoint_trajectory.cpp
#  src/plugins/sys_status.cpp
#  src/plugins/sys_time.cpp
#  src/plugins/vfr_hud.cpp
#  src/plugins/waypoint.cpp
#  src/plugins/wind_estimation.cpp
#  )
#add_dependencies(mavros_plugins
#  mavros
#  )
#target_link_libraries(mavros_plugins
#  mavros
#  ${catkin_LIBRARIES}
#  )
#pluginlib_export_plugin_description_file(mavros_plugins mavros_plugins.xml)

add_executable(mavros_node src/mavros_node.cpp)
target_link_libraries(mavros_node mavros)

# install(FILES
#   mavros_plugins.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(TARGETS mavros mavros_node #mavros_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # NOTE(vooon): Does not support our custom triple-license, tiered to make it to work.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(libmavros-frame-conversions-test test/test_frame_conversions.cpp)
  target_link_libraries(libmavros-frame-conversions-test mavros)
  #ament_target_dependencies(libmavros-frame-conversions-test mavros)

  ament_add_gtest(libmavros-sensor-orientation-test test/test_sensor_orientation.cpp)
  target_link_libraries(libmavros-sensor-orientation-test mavros)
  #ament_target_dependencies(libmavros-sensor-orientation-test mavros)

  ament_add_gtest(libmavros-quaternion-utils-test test/test_quaternion_utils.cpp)
  target_link_libraries(libmavros-quaternion-utils-test mavros)
  #ament_target_dependencies(libmavros-quaternion-utils-test mavros)

  ament_add_gmock(mavros-router-test test/test_router.cpp)
  target_link_libraries(mavros-router-test mavros)
  ament_target_dependencies(mavros-router-test mavros_msgs)
endif()

#ament_export_dependencies(console_bridge)
ament_export_include_directories(include)
ament_export_libraries(mavros)
#ament_export_targets(mavros_node)
ament_package()

# vim: ts=2 sw=2 et:

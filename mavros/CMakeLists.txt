cmake_minimum_required(VERSION 3.5)
project(mavros)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcomment")

# Allow GNU extensions (-std=gnu++20)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# find mavros dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(message_filters REQUIRED)

find_package(mavlink REQUIRED)
find_package(libmavconn REQUIRED)
find_package(console_bridge REQUIRED)
#find_package(rosconsole_bridge REQUIRED)  # XXX TODO: connect libmavconn loggers

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

## Find GeographicLib
# Append to CMAKE_MODULE_PATH since debian/ubuntu installs
# FindGeographicLib.cmake in a nonstand location
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

find_package(angles REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

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
  ${GeographicLib_INCLUDE_DIRS}
)

## Check if the datasets are installed
include(CheckGeographicLibDatasets)

if(rclcpp_VERSION VERSION_LESS 9.0.0)
  add_definitions(
    -DUSE_OLD_DECLARE_PARAMETER
  )
endif()

# [[[cog:
# import mavros_cog
# ]]]
# [[[end]]] (checksum: d41d8cd98f00b204e9800998ecf8427e)

add_library(mavros SHARED
  # [[[cog:
  # mavros_cog.outl_glob_files('src/lib')
  # ]]]
  src/lib/enum_sensor_orientation.cpp
  src/lib/enum_to_string.cpp
  src/lib/ftf_frame_conversions.cpp
  src/lib/ftf_quaternion_utils.cpp
  src/lib/mavros_router.cpp
  src/lib/mavros_uas.cpp
  src/lib/plugin.cpp
  src/lib/uas_ap.cpp
  src/lib/uas_data.cpp
  src/lib/uas_executor.cpp
  src/lib/uas_stringify.cpp
  src/lib/uas_tf.cpp
  src/lib/uas_timesync.cpp
  # [[[end]]] (checksum: 3239d16fc87982964694f18425f297b1)
)
ament_target_dependencies(mavros
  rclcpp
  rclcpp_components
  rcpputils
  #class_loader
  sensor_msgs
  pluginlib
  mavros_msgs
  libmavconn
  #console_bridge
  diagnostic_updater
  tf2_ros
  tf2_eigen
  Eigen3
)
target_link_libraries(mavros ${GeographicLib_LIBRARIES})
rclcpp_components_register_nodes(mavros "mavros::router::Router" "mavros::uas::UAS")

add_library(mavros_plugins SHARED
  # [[[cog:
  # mavros_cog.outl_glob_files('src/plugins')
  # ]]]
  src/plugins/actuator_control.cpp
  src/plugins/altitude.cpp
  src/plugins/command.cpp
  src/plugins/dummy.cpp
  src/plugins/ftp.cpp
  src/plugins/geofence.cpp
  src/plugins/global_position.cpp
  src/plugins/home_position.cpp
  src/plugins/imu.cpp
  src/plugins/local_position.cpp
  src/plugins/manual_control.cpp
  src/plugins/mission_protocol_base.cpp
  src/plugins/nav_controller_output.cpp
  src/plugins/param.cpp
  src/plugins/rallypoint.cpp
  src/plugins/rc_io.cpp
  src/plugins/setpoint_accel.cpp
  src/plugins/setpoint_attitude.cpp
  src/plugins/setpoint_position.cpp
  src/plugins/setpoint_raw.cpp
  src/plugins/setpoint_trajectory.cpp
  src/plugins/setpoint_velocity.cpp
  src/plugins/sys_status.cpp
  src/plugins/sys_time.cpp
  src/plugins/waypoint.cpp
  src/plugins/wind_estimation.cpp
  # [[[end]]] (checksum: ccf56c1a56e9dccf8464483f7b1eab99)
)
add_dependencies(mavros_plugins
  mavros
)
target_link_libraries(mavros_plugins
  mavros
)
ament_target_dependencies(mavros_plugins
  angles
  geometry_msgs
  geographic_msgs
  mavros_msgs
  std_msgs
  std_srvs
  sensor_msgs
  pluginlib
  nav_msgs
  trajectory_msgs
  rclcpp
  rclcpp_components
  rcpputils
  libmavconn
  diagnostic_updater
  tf2_ros
  tf2_eigen
  message_filters
  Eigen3
)
pluginlib_export_plugin_description_file(mavros mavros_plugins.xml)

add_executable(mavros_node src/mavros_node.cpp)
target_link_libraries(mavros_node mavros)

ament_python_install_package(${PROJECT_NAME})

# NOTE(vooon): for some reason console_scripts do not work with ament_python!
install(PROGRAMS scripts/mav
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mavros mavros_node mavros_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/install_geographiclib_datasets.sh
  DESTINATION lib/${PROJECT_NAME}
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
  find_package(ament_cmake_pytest REQUIRED)

  # NOTE(vooon): explicit call to find dependencies for ament_lint
  # ament_find_gtest()
  # ament_find_gmock()

  # NOTE(vooon): without that cppcheck fails on tests cpp files
  # list(APPEND ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS "${GMOCK_INCLUDE_DIRS}")
  # list(APPEND ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS "${GTEST_INCLUDE_DIRS}")
  # list(APPEND ament_cmake_cppcheck_ADDITIONAL_INCLUDE_DIRS "${EIGEN3_INCLUDE_DIRS}")
  # set(ament_cmake_cppcheck_LANGUAGE "c++")

  # NOTE(vooon): tired to provide more and more macro
  list(APPEND ament_cmake_cppcheck_ADDITIONAL_EXCLUDE "./test/*.cpp")

  # NOTE(vooon): cpplint complains about generated lines too long. Nothing more.
  # XXX(vooon): don't work for Foxy!
  # list(APPEND ament_cmake_cpplint_ADDITIONAL_EXCLUDE "./src/lib/enum_to_string.cpp")

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

  ament_add_gmock(mavros-uas-test test/test_uas.cpp)
  target_link_libraries(mavros-uas-test mavros)
  ament_target_dependencies(mavros-uas-test mavros_msgs)

  ament_add_pytest_test(mavros_py_test test/mavros_py
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
  )
endif()

#ament_export_dependencies(console_bridge)
ament_export_include_directories(include)
ament_export_libraries(mavros)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
#ament_export_targets(mavros_node)
ament_package()

# vim: ts=2 sw=2 et:

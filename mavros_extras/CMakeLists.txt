cmake_minimum_required(VERSION 3.5)
project(mavros_extras)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

# find mavros dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(message_filters REQUIRED)

find_package(mavlink REQUIRED)
find_package(mavros REQUIRED)
find_package(libmavconn REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(yaml_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

## Find GeographicLib
# Append to CMAKE_MODULE_PATH since debian/ubuntu installs
# FindGeographicLib.cmake in a nonstand location
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

find_package(angles REQUIRED)
find_package(eigen_stl_containers REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

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
find_package(visualization_msgs REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

include_directories(
  ${mavlink_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

if(rclcpp_VERSION VERSION_LESS 9.0.0)
  add_definitions(
    -DUSE_OLD_DECLARE_PARAMETER
  )
endif()


# [[[cog:
# import mavros_cog
# ]]]
# [[[end]]] (checksum: d41d8cd98f00b204e9800998ecf8427e)

add_library(mavros_extras_plugins SHARED
  # [[[cog:
  # mavros_cog.outl_glob_files('src/plugins')
  # ]]]
  src/plugins/3dr_radio.cpp
  src/plugins/adsb.cpp
  src/plugins/cam_imu_sync.cpp
  src/plugins/camera.cpp
  src/plugins/cellular_status.cpp
  src/plugins/companion_process_status.cpp
  src/plugins/debug_value.cpp
  src/plugins/distance_sensor.cpp
  src/plugins/esc_status.cpp
  src/plugins/esc_telemetry.cpp
  src/plugins/fake_gps.cpp
  src/plugins/gps_input.cpp
  src/plugins/gps_rtk.cpp
  src/plugins/gps_status.cpp
  src/plugins/guided_target.cpp
  src/plugins/hil.cpp
  src/plugins/landing_target.cpp
  src/plugins/log_transfer.cpp
  src/plugins/mag_calibration_status.cpp
  src/plugins/mocap_pose_estimate.cpp
  src/plugins/mount_control.cpp
  src/plugins/obstacle_distance.cpp
  src/plugins/odom.cpp
  src/plugins/onboard_computer_status.cpp
  src/plugins/optical_flow.cpp
  src/plugins/play_tune.cpp
  src/plugins/px4flow.cpp
  src/plugins/rangefinder.cpp
  src/plugins/terrain.cpp
  src/plugins/trajectory.cpp
  src/plugins/tunnel.cpp
  src/plugins/vfr_hud.cpp
  src/plugins/vibration.cpp
  src/plugins/vision_pose_estimate.cpp
  src/plugins/vision_speed_estimate.cpp
  src/plugins/wheel_odometry.cpp
  # [[[end]]] (checksum: e9b770ce685f15417fe9a859f1157b88)
)
ament_target_dependencies(mavros_extras_plugins
  angles
  geometry_msgs
  geographic_msgs
  mavros
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
  yaml_cpp_vendor
)
pluginlib_export_plugin_description_file(mavros mavros_plugins.xml)

add_library(mavros_extras SHARED
  # [[[cog:
  # mavros_cog.outl_glob_files('src/lib')
  # ]]]
  src/lib/servo_state_publisher.cpp
  # [[[end]]] (checksum: a3ce43c71c567f697861bcbcd0f25aa3)
)
ament_target_dependencies(mavros_extras
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  mavros_msgs
  #console_bridge
  yaml_cpp_vendor
  urdf
)
rclcpp_components_register_node(mavros_extras PLUGIN "mavros::extras::ServoStatePublisher" EXECUTABLE servo_state_publisher)

install(TARGETS mavros_extras mavros_extras_plugins
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
endif()

#ament_export_dependencies(console_bridge)
ament_export_include_directories(include)
ament_export_libraries(mavros_extras)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
#ament_export_targets(mavros_node)
ament_package()

# vim: ts=2 sw=2 et:

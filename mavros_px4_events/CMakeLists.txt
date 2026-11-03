cmake_minimum_required(VERSION 3.19)
project(mavros_px4_events)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(libmavconn REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(LibLZMA REQUIRED)

# MAVROS public headers include GeographicLib; its finder is installed here.
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

if(rclcpp_VERSION VERSION_LESS 17.0.0)
  # Humble and older use the .h tf2_ros headers in MAVROS public headers.
  add_compile_definitions(USE_OLD_TF2_ROS)
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libevents/libs/cpp/CMakeLists.txt")
  message(FATAL_ERROR
    "libevents submodule is missing. Run git submodule update --init --recursive.")
endif()

# Keep upstream libraries static and suitable for the plugin shared library.
# EXCLUDE_FROM_ALL excludes unrelated upstream targets and installation rules.
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(libevents/libs/cpp libevents_build EXCLUDE_FROM_ALL)
target_include_directories(libevents_parser SYSTEM INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libevents/libs/cpp>"
)

add_library(mavros_px4_events_plugins SHARED src/plugins/px4_events.cpp)
target_include_directories(mavros_px4_events_plugins PRIVATE
  ${mavros_INCLUDE_DIRS}
  ${libmavconn_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)
target_link_libraries(mavros_px4_events_plugins PRIVATE
  ${mavros_LIBRARIES}
  ${mavros_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  pluginlib::pluginlib
  rclcpp::rclcpp
  rcpputils::rcpputils
  tf2_eigen::tf2_eigen
  tf2_ros::tf2_ros
  libevents_parser
  LibLZMA::LibLZMA
)
pluginlib_export_plugin_description_file(mavros mavros_plugins.xml)

install(TARGETS mavros_px4_events_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES libevents/LICENSE.md
  DESTINATION share/${PROJECT_NAME}
  RENAME LICENSE.libevents
)

ament_package()

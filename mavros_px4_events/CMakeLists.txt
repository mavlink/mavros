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

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vendor/libevents/libs/cpp/CMakeLists.txt")
  message(WARNING
    "Skipping mavros_px4_events: libevents submodule is missing. "
    "Run git submodule update --init --recursive "
    "mavros_px4_events/vendor/libevents from the repository root, then rebuild.")
  ament_package()
  return()
endif()

find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(libmavconn REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
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
  # Humble and older use the legacy service QoS overload and tf2_ros headers.
  add_compile_definitions(USE_OLD_RMW_QOS USE_OLD_TF2_ROS)
endif()

# Keep upstream libraries static and suitable for the plugin shared library.
# EXCLUDE_FROM_ALL excludes unrelated upstream targets and installation rules.
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(vendor/libevents/libs/cpp libevents_build EXCLUDE_FROM_ALL)
target_include_directories(libevents_parser SYSTEM INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/libevents/libs/cpp>"
)

add_library(mavros_px4_events_plugins SHARED src/plugins/px4_events.cpp)
target_sources(mavros_px4_events_plugins PRIVATE
  src/plugins/px4_events/acquisition.cpp
  src/plugins/px4_events/cache.cpp
  src/plugins/px4_events/decoder.cpp
  src/plugins/px4_events/discovery.cpp
  src/plugins/px4_events/metadata.cpp
  src/plugins/px4_events/xz_metadata.cpp
)
pluginlib_export_plugin_description_file(mavros mavros_plugins.xml)

set(px4_events_targets mavros_px4_events_plugins)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Threads REQUIRED)

  # This test includes decoder.cpp to exercise its private formatting state.
  ament_add_gtest(mavros-px4-events-test
    test/test_px4_events.cpp
    src/plugins/px4_events/xz_metadata.cpp
  )
  target_compile_definitions(mavros-px4-events-test PRIVATE
    TEST_EVENT_METADATA="${CMAKE_CURRENT_SOURCE_DIR}/test/px4_events.json"
  )

  ament_add_gtest(mavros-px4-event-metadata-test
    test/test_px4_event_metadata.cpp
    src/plugins/px4_events/metadata.cpp
  )

  ament_add_gtest(mavros-px4-event-cache-test
    test/test_px4_event_cache.cpp
    src/plugins/px4_events/cache.cpp
    src/plugins/px4_events/metadata.cpp
    src/plugins/px4_events/decoder.cpp
    src/plugins/px4_events/xz_metadata.cpp
  )
  target_link_libraries(mavros-px4-event-cache-test Threads::Threads)

  ament_add_gtest(mavros-px4-event-discovery-test
    test/test_px4_event_discovery.cpp
    src/plugins/px4_events/discovery.cpp
    src/plugins/px4_events/metadata.cpp
  )

  ament_add_gtest(mavros-px4-event-acquisition-test
    test/test_px4_event_acquisition.cpp
    src/plugins/px4_events/acquisition.cpp
    src/plugins/px4_events/cache.cpp
    src/plugins/px4_events/metadata.cpp
    src/plugins/px4_events/decoder.cpp
    src/plugins/px4_events/xz_metadata.cpp
  )

  ament_add_gtest(mavros-px4-event-plugin-test
    test/test_px4_events_plugin.cpp
    src/plugins/px4_events/acquisition.cpp
    src/plugins/px4_events/cache.cpp
    src/plugins/px4_events/decoder.cpp
    src/plugins/px4_events/discovery.cpp
    src/plugins/px4_events/metadata.cpp
    src/plugins/px4_events/xz_metadata.cpp
  )
  target_compile_definitions(mavros-px4-event-plugin-test PRIVATE
    TEST_EVENT_METADATA="${CMAKE_CURRENT_SOURCE_DIR}/test/px4_events.json"
  )

  list(APPEND px4_events_targets
    mavros-px4-events-test
    mavros-px4-event-metadata-test
    mavros-px4-event-cache-test
    mavros-px4-event-discovery-test
    mavros-px4-event-acquisition-test
    mavros-px4-event-plugin-test
  )
endif()

# The plugin and helper tests use the same dependency/header environment.
foreach(target IN LISTS px4_events_targets)
  # Humble's ament_add_gtest uses the plain link signature; do not mix signatures.
  set(link_scope "")
  if(target STREQUAL "mavros_px4_events_plugins")
    set(link_scope PRIVATE)
  endif()
  target_include_directories(${target} PRIVATE
    ${mavros_INCLUDE_DIRS}
    ${libmavconn_INCLUDE_DIRS}
    ${mavlink_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
  )
  target_link_libraries(${target} ${link_scope}
    ${mavros_LIBRARIES}
    ${mavros_msgs_TARGETS}
    ${rcl_interfaces_TARGETS}
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
endforeach()

install(TARGETS mavros_px4_events_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES vendor/libevents/LICENSE.md
  DESTINATION share/${PROJECT_NAME}
  RENAME LICENSE.libevents
)

ament_package()

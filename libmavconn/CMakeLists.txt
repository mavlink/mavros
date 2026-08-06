cmake_minimum_required(VERSION 3.10)
project(libmavconn)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")
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

# add package modules path, not needed in dependent packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

## System dependencies are found with CMake's conventions
find_package(console_bridge REQUIRED)
find_package(ASIO REQUIRED)
find_package(mavlink REQUIRED)

include(em_expand)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/generated/include
  ${ASIO_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(mavconn SHARED
  ${CMAKE_CURRENT_BINARY_DIR}/generated/src/mavlink_helpers.cpp
  src/interface.cpp
  src/serial.cpp
  src/tcp.cpp
  src/udp.cpp
)
target_link_libraries(mavconn PUBLIC
  console_bridge::console_bridge
)

# Use em_expand macros to generate source files
em_expand(${CMAKE_CURRENT_SOURCE_DIR}/mavlink.context.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/mavlink.context.py
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mavconn/mavlink_dialect.hpp.em
  ${CMAKE_CURRENT_BINARY_DIR}/generated/include/mavconn/mavlink_dialect.hpp)

em_expand(${CMAKE_CURRENT_SOURCE_DIR}/mavlink.context.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/mavlink.context.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mavlink_helpers.cpp.em
  ${CMAKE_CURRENT_BINARY_DIR}/generated/src/mavlink_helpers.cpp)

message(STATUS "Copy mavlink_dialect.hpp to source tree")
file(
  COPY ${CMAKE_CURRENT_BINARY_DIR}/generated/include/mavconn/mavlink_dialect.hpp
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/include/mavconn/
)

install(TARGETS mavconn
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated/include/mavconn/
  DESTINATION include/mavconn
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # NOTE(vooon): i prefer black style, which uses double quotes
  set(ament_cmake_flake8_CONFIG_FILE "./setup.cfg")

  # NOTE(vooon): Does not support our custom triple-license, tiered to make it to work.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)

  # Old releases has different uncrustify config, disable for < Jazzy
  if(rclcpp_VERSION VERSION_LESS 22.0.0)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  endif()

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_mavconn test/test_mavconn.cpp)
  target_link_libraries(test_mavconn
    mavconn
    console_bridge::console_bridge
  )

  find_package(ament_cmake_google_benchmark REQUIRED)
  # Standalone transport pair benchmark (udp/tcp). Not registered as a CI test:
  # the async close/shutdown race makes it intermittently hang under the test
  # harness. Run manually to check for mavconn hot-path regressions:
  #   build/libmavconn/mavconn-benchmark
  ament_add_google_benchmark_executable(mavconn-benchmark test/test_mavconn_benchmark.cpp)
  target_link_libraries(mavconn-benchmark
    mavconn
    console_bridge::console_bridge
  )

  # pymavlink-referenced byte-stream e2e test.
  # Small ctypes shim over MAVConnUDP; the pytest skips cleanly when pymavlink
  # (a pip package) is not installed, so CI/buildfarm stays green.
  add_library(mavconn_e2e_shim SHARED test/e2e/mavconn_e2e_shim.cpp)
  target_link_libraries(mavconn_e2e_shim
    mavconn
    console_bridge::console_bridge
  )

  ament_add_pytest_test(libmavconn_pymavlink_e2e test/e2e
    ENV
      MAVCONN_E2E_SHIM=$<TARGET_FILE:mavconn_e2e_shim>
    APPEND_ENV
      LD_LIBRARY_PATH=$<TARGET_FILE_DIR:mavconn_e2e_shim>
  )

endif()

ament_export_dependencies(console_bridge)
ament_export_include_directories(include)
ament_export_libraries(mavconn)
#ament_export_targets(mavconn)
ament_package(
  CONFIG_EXTRAS "libmavconn-extras.cmake"
)

# vim: ts=2 sw=2 et:

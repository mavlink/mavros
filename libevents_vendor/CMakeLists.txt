cmake_minimum_required(VERSION 3.19)
project(libevents_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

set(libevents_revision 0ef305faa4cffdb09f01ca862ebfc108b7ff0cd5)
set(libevents_install_dir "${CMAKE_CURRENT_BINARY_DIR}/libevents_install")
set(libevents_cmake_args)
if(DEFINED CMAKE_BUILD_TYPE)
  list(APPEND libevents_cmake_args "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
endif()
if(CMAKE_TOOLCHAIN_FILE)
  list(APPEND libevents_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
endif()

ExternalProject_Add(libevents_external
  URL "https://codeload.github.com/mavlink/libevents/tar.gz/${libevents_revision}"
  URL_HASH SHA256=a0f0e3b04967d13c0f3763eefcbf024fbd3de2afb6a39f5307f5fc1b4aca8990
  SOURCE_SUBDIR libs/cpp
  UPDATE_COMMAND ""
  TIMEOUT 300
  CMAKE_ARGS
    "-DCMAKE_INSTALL_PREFIX=${libevents_install_dir}"
    -DCMAKE_INSTALL_LIBDIR=lib
    -DCMAKE_INSTALL_INCLUDEDIR=include
    "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
    "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DBUILD_SHARED_LIBS=OFF
    -DENABLE_TESTING=OFF
    ${libevents_cmake_args}
)

# Use the same bundled JSON revision as libevents for metadata validation.
ExternalProject_Get_Property(libevents_external SOURCE_DIR)
install(FILES "${SOURCE_DIR}/libs/cpp/parse/nlohmann/json.hpp"
  DESTINATION opt/${PROJECT_NAME}/include/libevents/parse/nlohmann
)

install(DIRECTORY "${libevents_install_dir}/"
  DESTINATION opt/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
install(FILES README.md LICENSE.libevents LICENSE.nlohmann-json
  DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS "libevents_vendor-extras.cmake")

cmake_minimum_required(VERSION 3.10)
project(xz_embedded_vendor)

find_package(ament_cmake REQUIRED)
include(ExternalProject)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

set(xz_embedded_revision ae63ae3a36ed01724674e8f3d750dc47bf125410)
set(xz_embedded_install_dir "${CMAKE_CURRENT_BINARY_DIR}/xz_embedded_install")
set(xz_embedded_cmake_args)
if(DEFINED CMAKE_BUILD_TYPE)
  list(APPEND xz_embedded_cmake_args "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
endif()
if(CMAKE_TOOLCHAIN_FILE)
  list(APPEND xz_embedded_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
endif()

# Upstream supplies C sources and a userspace Makefile, but no CMake project.
ExternalProject_Add(xz_embedded_external
  URL "https://codeload.github.com/tukaani-project/xz-embedded/tar.gz/${xz_embedded_revision}"
  URL_HASH SHA256=2dd3ad60d2a1c3fe39948bf977ddb977bc84bb92cc5f3b20b0dcae8faa619703
  UPDATE_COMMAND ""
  TIMEOUT 300
  CONFIGURE_COMMAND "${CMAKE_COMMAND}"
    -S "${CMAKE_CURRENT_SOURCE_DIR}/upstream"
    -B <BINARY_DIR>
    "-DXZ_EMBEDDED_SOURCE_DIR=<SOURCE_DIR>"
    "-DCMAKE_INSTALL_PREFIX=${xz_embedded_install_dir}"
    "-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
    ${xz_embedded_cmake_args}
  BUILD_COMMAND "${CMAKE_COMMAND}" --build <BINARY_DIR>
  INSTALL_COMMAND "${CMAKE_COMMAND}" --build <BINARY_DIR> --target install
)

install(DIRECTORY "${xz_embedded_install_dir}/"
  DESTINATION opt/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
install(FILES README.md LICENSE.xz-embedded
  DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS "xz_embedded_vendor-extras.cmake")

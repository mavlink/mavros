cmake_minimum_required(VERSION 2.8.3)
project(libmavconn)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# add package modules path, not needed in dependend packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
include(EnableCXX11)
include(MavrosMavlink)

# Fixed in mavlink 2016.7.7
#-> enum values out of int range
#list(APPEND IGNORE_DIALECTS "autoquad")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavconn
  DEPENDS Boost console_bridge mavlink
  CFG_EXTRAS libmavconn-extras.cmake
)

###########
## Build ##
###########

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(mavconn
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/src/mavlink_helpers.cpp
  src/interface.cpp
  src/serial.cpp
  src/tcp.cpp
  src/udp.cpp
)
target_link_libraries(mavconn
  ${Boost_LIBRARIES}
  ${console_bridge_LIBRARIES}
)

# Use catkin-supplied em_expand macros to generate source files
em_expand(${CMAKE_CURRENT_SOURCE_DIR}/mavlink.context.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/mavlink.context.py
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mavconn/mavlink_dialect.h.em
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include/mavconn/mavlink_dialect.h)

em_expand(${CMAKE_CURRENT_SOURCE_DIR}/mavlink.context.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/mavlink.context.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mavlink_helpers.cpp.em
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/src/mavlink_helpers.cpp)

message(STATUS "Copy mavlink_dialect.h to source tree")
file(
  COPY ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include/mavconn/mavlink_dialect.h
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/include/mavconn/
  )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS mavconn
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/mavconn/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mavconn
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include/mavconn/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/mavconn
  FILES_MATCHING PATTERN "*.h"
)

## Install cmake files (thanks to cmake_modules package)
install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(mavconn-test test/test_mavconn.cpp)
  target_link_libraries(mavconn-test mavconn pthread)
endif()

# vim: ts=2 sw=2 et:

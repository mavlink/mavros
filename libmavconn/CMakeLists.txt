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

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mavconn
  DEPENDS Boost mavlink console_bridge
  CFG_EXTRAS libmavconn-extras.cmake
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(mavconn
  src/mavlink_helpers.cpp
  src/interface.cpp
  src/serial.cpp
  src/udp.cpp
  src/tcp.cpp
)
target_link_libraries(mavconn
  ${Boost_LIBRARIES}
  ${console_bridge_LIBRARIES}
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

## Install cmake files (thanks to cmake_modules package)
install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

catkin_add_gtest(mavconn-test test/test_mavconn.cpp)
target_link_libraries(mavconn-test mavconn)

endif()

# vim: ts=2 sw=2 et:

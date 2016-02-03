cmake_minimum_required(VERSION 2.8.3)
project(mavros_extras)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  cv_bridge
  mavros
  roscpp
  mavros_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  urdf
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

include(EnableCXX11)
include(MavrosMavlink)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS image_transport mavros mavros_msgs roscpp sensor_msgs geometry_msgs
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
)

add_library(mavros_extras
  src/plugins/distance_sensor.cpp
  src/plugins/px4flow.cpp
  src/plugins/image_pub.cpp
  src/plugins/mocap_pose_estimate.cpp
  src/plugins/vision_pose_estimate.cpp
  src/plugins/vision_speed_estimate.cpp
  src/plugins/vibration.cpp
  src/plugins/cam_imu_sync.cpp
)
target_link_libraries(mavros_extras
  ${mavros_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(gcs_image_bridge
  src/gcs_image_bridge.cpp
)
target_link_libraries(gcs_image_bridge
  ${mavros_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(copter_visualization
  src/copter_visualization.cpp
)
target_link_libraries(copter_visualization
  ${catkin_LIBRARIES}
)

add_executable(servo_state_publisher
  src/servo_state_publisher.cpp
)
target_link_libraries(servo_state_publisher
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
file(GLOB SCRIPTS ${PROJECT_SOURCE_DIR}/scripts/*)
install(PROGRAMS
  ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS mavros_extras gcs_image_bridge copter_visualization servo_state_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  mavros_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mavros_extras.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

# vim: ts=2 sw=2 et:

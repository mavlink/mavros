# This module includes MAVLink package and
# define MAVLINK_DIALECT required by mavconn_mavlink.h

find_package(mavlink REQUIRED)

# fallback for older mavlink package.
if(NOT DEFINED mavlink_DIALECTS)
  list(APPEND mavlink_DIALECTS "ardupilotmega")
  list(APPEND mavlink_DIALECTS "pixhawk")
  list(APPEND mavlink_DIALECTS "common")
endif()

# Select MAVLink dialect
set(MAVLINK_DIALECT "ardupilotmega" CACHE STRING "MAVLink dialect selector")
set_property(CACHE MAVLINK_DIALECT PROPERTY STRINGS ${mavlink_DIALECTS})

# TODO: check that selected dialect are known

message(STATUS "Selected MAVLink dialect: ${MAVLINK_DIALECT}")

# define dialect for mavconn_mavlink.h
add_definitions(
  -DMAVLINK_DIALECT=${MAVLINK_DIALECT}
)

# vim: set ts=2 sw=2 et:

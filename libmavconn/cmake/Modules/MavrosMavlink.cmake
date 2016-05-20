# This module includes MAVLink package and
# define MAVLINK_DIALECT required by mavconn_mavlink.h

find_package(mavlink REQUIRED)

# fallback for older mavlink package.
if (NOT DEFINED mavlink_DIALECTS)
  list(APPEND mavlink_DIALECTS "ardupilotmega")
  list(APPEND mavlink_DIALECTS "common")
endif ()

# Select MAVLink dialect
set(MAVLINK_DIALECT "ardupilotmega" CACHE STRING "MAVLink dialect selector")
set_property(CACHE MAVLINK_DIALECT PROPERTY STRINGS ${mavlink_DIALECTS})

# check that selected dialect are known
set(MAVLINK_DIALECT_KNOWN)
foreach (dialect ${mavlink_DIALECTS})
  if (MAVLINK_DIALECT STREQUAL dialect)
    set(MAVLINK_DIALECT_KNOWN TRUE)
  endif ()
endforeach ()

if (MAVLINK_DIALECT_KNOWN)
  message(STATUS "Selected MAVLink dialect: ${MAVLINK_DIALECT}")
else ()
  message(FATAL_ERROR "Unknown MAVLink dialect: ${MAVLINK_DIALECT}, known dialects: ${mavlink_DIALECTS}")
endif ()

# define dialect for mavconn_mavlink.h
add_definitions(
  -DMAVLINK_DIALECT=${MAVLINK_DIALECT}
)

# mavlink 2.0 capable mavgen produce little different API of mavlink 1.0
if (mavlink2_DIALECTS)
  add_definitions(
    -DMAVLINK2_COMPAT
  )
endif()

# vim: set ts=2 sw=2 et:

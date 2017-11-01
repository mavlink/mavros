##
# This module verifies the installation of the GeographicLib datasets and warns
# if it doesn't detect them.
##

find_path(GEOGRAPHICLIB_GEOID_PATH NAMES geoids PATH_SUFFIXES share/GeographicLib share/geographiclib)
find_path(GEOGRAPHICLIB_GRAVITY_PATH_ NAMES gravity PATH_SUFFIXES share/GeographicLib)
find_path(GEOGRAPHICLIB_MAGNETIC_PATH_ NAMES magnetic PATH_SUFFIXES share/GeographicLib)

if(NOT GEOGRAPHICLIB_GEOID_PATH)
  message(STATUS "No geoid model datasets found. This will result on a SIGINT! Please execute the script install_geographiclib_dataset.sh in /mavros/scripts")
else()
  message(STATUS "Geoid model datasets found in: " ${GEOGRAPHICLIB_GEOID_PATH}/geoid)
  set(GEOGRAPHICLIB_GEOID_PATH ${GEOGRAPHICLIB_GEOID_PATH}/geoid)
endif()
if(NOT GEOGRAPHICLIB_GRAVITY_PATH_)
  message(STATUS "No gravity field model datasets found. Please execute the script install_geographiclib_dataset.sh in /mavros/scripts")
else()
  message(STATUS "Gravity Field model datasets found in: " ${GEOGRAPHICLIB_GRAVITY_PATH_}/gravity)
  set(GEOGRAPHICLIB_GRAVITY_PATH ${GEOGRAPHICLIB_GRAVITY_PATH_}/gravity)
endif()
if(NOT GEOGRAPHICLIB_MAGNETIC_PATH_)
  message(STATUS "No magnetic field model datasets found. Please execute the script install_geographiclib_dataset.sh in /mavros/scripts")
else()
  message(STATUS "Magnetic Field model datasets found in: " ${GEOGRAPHICLIB_MAGNETIC_PATH_}/magnetic)
  set(GEOGRAPHICLIB_MAGNETIC_PATH ${GEOGRAPHICLIB_MAGNETIC_PATH_}/magnetic)
endif()

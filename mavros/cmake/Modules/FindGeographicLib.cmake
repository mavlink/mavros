# Used to find GeographicLib
# Thanks to: Kitware, Inc. repo

if( Geographiclib_DIR )
  find_package( GeographicLib NO_MODULE )
elseif( NOT GeographicLib_FOUND )
  include(CommonFindMacros)

  setup_find_root_context(GeographicLib)
  find_path( GeographicLib_INCLUDE_DIR GeographicLib/GeoCoords.hpp
    ${GeographicLib_FIND_OPTS})
  find_library( GeographicLib_LIBRARY
    NAMES Geographic GeographicLib Geographic_d GeographicLib_d
    ${GeographicLib_FIND_OPTS})
  restore_find_root_context(GeographicLib)

  include( FindPackageHandleStandardArgs )
  FIND_PACKAGE_HANDLE_STANDARD_ARGS( GeographicLib GeographicLib_INCLUDE_DIR GeographicLib_LIBRARY )
  if( GEOGRAPHICLIB_FOUND )
    set( GeographicLib_FOUND TRUE )
  endif()
endif()

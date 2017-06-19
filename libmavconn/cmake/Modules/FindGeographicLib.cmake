##
# Look for GeographicLib
# Modified version from the original so it can work for Debian-alike systems
# and find the proper shared library.
#
# Set:
#  GeographicLib_FOUND = GEOGRAPHICLIB_FOUND = TRUE
#  GeographicLib_INCLUDE_DIRS = /usr/local/include  or
#                               /usr/include
#  GeographicLib_LIBRARIES = /usr/local/lib/libGeographic.so  or
#                            /usr/lib/x86_64-linux-gnu/libGeographic.so
#  GeographicLib_LIBRARY_DIRS = /usr/local/lib  or
#                               /usr/lib/x86_64-linux-gnu/ for Debian systems
##

find_library (GeographicLib_LIBRARIES Geographic
  PATHS "${CMAKE_INSTALL_PREFIX}/../GeographicLib/lib")

if (GeographicLib_LIBRARIES)
  get_filename_component (GeographicLib_LIBRARY_DIRS
    "${GeographicLib_LIBRARIES}" PATH)

  get_filename_component (_ROOT_DIR "${GeographicLib_LIBRARY_DIRS}" PATH)
  set (GeographicLib_INCLUDE_DIRS "${_ROOT_DIR}/../include")
  set (GeographicLib_BINARY_DIRS "${_ROOT_DIR}/../bin")

  # Required for Debian-alike systems
  find_library (GeographicLib_LIBRARIES_DEBIAN libGeographic.so
    PATHS "/usr/lib/x86_64-linux-gnu")
  if (GeographicLib_LIBRARIES_DEBIAN)
    set (GeographicLib_LIBRARIES ${GeographicLib_LIBRARIES_DEBIAN})
    get_filename_component (GeographicLib_LIBRARY_DIRS
      "${GeographicLib_LIBRARIES}" PATH)
  endif ()

  if (NOT EXISTS "${GeographicLib_INCLUDE_DIRS}/GeographicLib/Config.h")
    unset (GeographicLib_INCLUDE_DIRS)
    unset (GeographicLib_LIBRARIES)
    unset (GeographicLib_LIBRARY_DIRS)
    unset (GeographicLib_BINARY_DIRS)
  endif ()
  unset (_ROOT_DIR)
endif ()

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (GeographicLib DEFAULT_MSG
  GeographicLib_LIBRARY_DIRS GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)
mark_as_advanced (GeographicLib_LIBRARY_DIRS GeographicLib_LIBRARIES
  GeographicLib_INCLUDE_DIRS)

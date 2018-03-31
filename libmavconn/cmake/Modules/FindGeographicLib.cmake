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

find_path (GeographicLib_INCLUDE_DIRS NAMES GeographicLib/Config.h)

find_library (GeographicLib_LIBRARIES NAMES Geographic)

# Required for Debian-alike systems
find_library (GeographicLib_LIBRARIES_DEBIAN libGeographic.so
  PATHS "/usr/lib/x86_64-linux-gnu")
if (GeographicLib_LIBRARIES_DEBIAN)
  set (GeographicLib_LIBRARIES ${GeographicLib_LIBRARIES_DEBIAN})
  get_filename_component (GeographicLib_LIBRARY_DIRS
    "${GeographicLib_LIBRARIES}" PATH)
endif ()

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (GeographicLib DEFAULT_MSG
  GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)
mark_as_advanced (GeographicLib_LIBRARIES GeographicLib_INCLUDE_DIRS)

#message(WARNING "GL: F:${GeographicLib_FOUND} L:${GeographicLib_LIBRARIES} I:${GeographicLib_INCLUDE_DIRS}")

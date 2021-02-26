#
# libmavconn
# Copyright 2013-2016,2018,2021 Vladimir Ermakov, All rights reserved.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
#

# Look for GeographicLib
#
# Set
#  GEOGRAPHICLIB_FOUND = TRUE
#  GeographicLib_INCLUDE_DIRS = /usr/local/include
#  GeographicLib_LIBRARIES = /usr/local/lib/libGeographic.so
#  GeographicLib_LIBRARY_DIRS = /usr/local/lib

find_path(GEOGRAPHICLIB_INCLUDE_DIRS NAMES GeographicLib/Config.h)

find_library(GEOGRAPHICLIB_LIBRARIES NAMES Geographic)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GEOGRAPHICLIB DEFAULT_MSG
  GEOGRAPHICLIB_LIBRARIES GEOGRAPHICLIB_INCLUDE_DIRS)
mark_as_advanced(GEOGRAPHICLIB_LIBRARIES GEOGRAPHICLIB_INCLUDE_DIRS)

#message(WARNING "GL: F:${GEOGRAPHICLIB_FOUND} L:${GEOGRAPHICLIB_LIBRARIES} I:${GEOGRAPHICLIB_INCLUDE_DIRS}")

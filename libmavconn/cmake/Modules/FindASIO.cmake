#
# libmavconn
# Copyright 2013-2016,2018,2021 Vladimir Ermakov, All rights reserved.
#
# This file is part of the mavros package and subject to the license terms
# in the top-level LICENSE file of the mavros repository.
# https://github.com/mavlink/mavros/tree/master/LICENSE.md
#

# Look for Asio
#
# Set
#  Asio_FOUND = TRUE
#  Asio_INCLUDE_DIRS = /usr/local/include

find_path(ASIO_INCLUDE_DIRS
  NAMES asio.hpp
  PATHS /usr/include /usr/local/include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ASIO DEFAULT_MSG
  ASIO_INCLUDE_DIRS
)
mark_as_advanced(ASIO_INCLUDE_DIRS)

# Look for Asio
#
# Set
#  Asio_FOUND = TRUE
#  Asio_INCLUDE_DIRS = /usr/local/include

find_path(Asio_INCLUDE_DIRS
  NAMES asio.hpp
  PATHS /usr/include /usr/local/include
)

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (Asio DEFAULT_MSG
  Asio_INCLUDE_DIRS
)
mark_as_advanced (Asio_INCLUDE_DIRS)

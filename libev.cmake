# current libev don't provide any find-package or pkg-config files
# so we just find library and header.

find_library(libev_LIBRARY ev)
find_path(libev_INCLUDE_DIR ev++.h
	PATH_SUFFIXES include/ev include)
find_package_handle_standard_args(libev DEFAULT_MSG libev_LIBRARY libev_INCLUDE_DIR)
mark_as_advanced(libev_INCLUDE_DIR libev_LIBRARY)

if (NOT libev_FOUND)
	message(STATUS "The libev package not found. please install libev-dev")
endif()

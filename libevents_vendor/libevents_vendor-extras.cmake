get_filename_component(_libevents_vendor_prefix
  "${CMAKE_CURRENT_LIST_DIR}/../../../opt/libevents_vendor" ABSOLUTE
)

if(NOT TARGET libevents_parser)
  include("${_libevents_vendor_prefix}/lib/cmake/libevents/libeventsConfig.cmake")
endif()

# Upstream exports the parser library without its header search path.
if(NOT TARGET libevents_vendor::parser)
  add_library(libevents_vendor::parser INTERFACE IMPORTED)
  set_target_properties(libevents_vendor::parser PROPERTIES
    INTERFACE_LINK_LIBRARIES libevents_parser
    INTERFACE_INCLUDE_DIRECTORIES "${_libevents_vendor_prefix}/include"
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_libevents_vendor_prefix}/include"
  )
endif()

unset(_libevents_vendor_prefix)

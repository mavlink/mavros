get_filename_component(_xz_embedded_vendor_prefix
  "${CMAKE_CURRENT_LIST_DIR}/../../../opt/xz_embedded_vendor" ABSOLUTE
)

if(NOT TARGET xz_embedded_vendor::decoder)
  add_library(xz_embedded_vendor::decoder STATIC IMPORTED)
  set_target_properties(xz_embedded_vendor::decoder PROPERTIES
    IMPORTED_LOCATION "${_xz_embedded_vendor_prefix}/lib/libxz_embedded.a"
    INTERFACE_INCLUDE_DIRECTORIES "${_xz_embedded_vendor_prefix}/include"
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_xz_embedded_vendor_prefix}/include"
    INTERFACE_COMPILE_DEFINITIONS "XZ_USE_CRC64;XZ_USE_SHA256"
  )
endif()

unset(_xz_embedded_vendor_prefix)

cmake_minimum_required(VERSION 3.10)
project(xz_embedded LANGUAGES C)

add_library(xz_embedded STATIC
  "${XZ_EMBEDDED_SOURCE_DIR}/linux/lib/xz/xz_crc32.c"
  "${XZ_EMBEDDED_SOURCE_DIR}/linux/lib/xz/xz_crc64.c"
  "${XZ_EMBEDDED_SOURCE_DIR}/linux/lib/xz/xz_sha256.c"
  "${XZ_EMBEDDED_SOURCE_DIR}/linux/lib/xz/xz_dec_lzma2.c"
  "${XZ_EMBEDDED_SOURCE_DIR}/linux/lib/xz/xz_dec_stream.c"
)
target_include_directories(xz_embedded PRIVATE
  "${XZ_EMBEDDED_SOURCE_DIR}/linux/include/linux"
  "${XZ_EMBEDDED_SOURCE_DIR}/userspace"
)
target_compile_definitions(xz_embedded PRIVATE
  XZ_DEC_SINGLE XZ_USE_CRC64 XZ_USE_SHA256
)
set_target_properties(xz_embedded PROPERTIES
  C_STANDARD 99
  C_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE ON
  C_VISIBILITY_PRESET hidden
)

install(TARGETS xz_embedded ARCHIVE DESTINATION lib)
install(FILES "${XZ_EMBEDDED_SOURCE_DIR}/linux/include/linux/xz.h"
  DESTINATION include/xz_embedded
)

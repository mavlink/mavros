# Setup restricted search paths
macro(setup_find_root_context PKG)
  if(${PKG}_ROOT)
    set(_CMAKE_FIND_ROOT_PATH "${CMAKE_FIND_ROOT_PATH}")
    set(_CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
    set(CMAKE_FIND_ROOT_PATH "${${PKG}_ROOT}")
    set(CMAKE_PREFIX_PATH /)
    set(_${PKG}_FIND_OPTS ${${PKG}_FIND_OPTS})
    set(${PKG}_FIND_OPTS ONLY_CMAKE_FIND_ROOT_PATH ${${PKG}_FIND_OPTS})
  endif()
endmacro()


# Restore original search paths
macro(restore_find_root_context PKG)
  if(${PKG}_ROOT)
    set(CMAKE_FIND_ROOT_PATH "${_CMAKE_FIND_ROOT_PATH}")
    set(CMAKE_PREFIX_PATH "${_CMAKE_PREFIX_PATH}")
    set(${PKG}_FIND_OPTS ${_${PKG}_FIND_OPTS})
  endif()
endmacro()

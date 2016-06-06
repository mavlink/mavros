# This module enables C++11 or C++14 support
#
# thanks for: http://www.guyrutenberg.com/2014/01/05/enabling-c11-c0x-in-cmake/

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_STD_CXX14)
if (NOT COMPILER_SUPPORTS_STD_CXX14)
  check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_STD_CXX11)
endif ()

if (COMPILER_SUPPORTS_STD_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif (COMPILER_SUPPORTS_STD_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else ()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

# vim: set ts=2 sw=2 et:

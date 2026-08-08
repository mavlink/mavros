cmake_minimum_required(VERSION 3.16)
project(mavros_tools_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- yaml-cpp (JSON output) ---
# Humble typically provides yaml-cpp via yaml_cpp_vendor.
# Newer distros may provide a system CMake config target.
if(DEFINED ENV{ROS_DISTRO} AND "$ENV{ROS_DISTRO}" STREQUAL "humble")
  find_package(yaml_cpp_vendor REQUIRED)
  set(YAMLCPP_LIB yaml-cpp)
else()
  find_package(yaml-cpp QUIET CONFIG)
  if(yaml-cpp_FOUND)
    set(YAMLCPP_LIB yaml-cpp::yaml-cpp)
  else()
    find_package(yaml_cpp_vendor QUIET)
    if(yaml_cpp_vendor_FOUND)
      set(YAMLCPP_LIB yaml-cpp)
    else()
      find_package(PkgConfig REQUIRED)
      pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
      list(APPEND YAMLCPP_LIB ${YAMLCPP_LIBRARIES})
    endif()
  endif()
endif()

# --- clang tooling ---
# Requires clang/libclang-dev installed (provides llvm-config + ClangConfig.cmake).
# gendoc.sh passes -DLLVM_ROOT=<root of the LLVM install, e.g. /usr/lib/llvm-21>
# so find_package(LLVM/Clang) can locate the config files.
if(LLVM_ROOT)
  list(APPEND CMAKE_PREFIX_PATH "${LLVM_ROOT}/lib/cmake/llvm")
  list(APPEND CMAKE_PREFIX_PATH "${LLVM_ROOT}/lib/cmake/clang")
endif()

find_package(LLVM REQUIRED CONFIG)
message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
find_package(Clang REQUIRED CONFIG)

add_executable(plugin_doc_extract plugin_doc_extract.cpp)

target_include_directories(plugin_doc_extract PRIVATE ${LLVM_INCLUDE_DIRS})
target_link_libraries(plugin_doc_extract PRIVATE
  clangTooling
  clangFrontend
  clangSerialization
  clangDriver
  clangParse
  clangSema
  clangAnalysis
  clangEdit
  clangAST
  clangLex
  clangBasic
  LLVMSupport
  LLVMOption
  LLVMXRay
  ${YAMLCPP_LIB}
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(plugin_doc_extract PRIVATE -O3 -DNDEBUG -Wall -Wextra -Wpedantic)
endif()
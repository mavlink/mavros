cmake_minimum_required(VERSION 3.16)
project(mavros_tools_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(yaml-cpp QUIET CONFIG)

add_executable(plugin_doc_gen_cpp plugin_doc_extract.cpp)
if(yaml-cpp_FOUND)
  target_link_libraries(plugin_doc_gen_cpp PRIVATE yaml-cpp::yaml-cpp)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
  target_include_directories(plugin_doc_gen_cpp PRIVATE ${YAMLCPP_INCLUDE_DIRS})
  target_link_directories(plugin_doc_gen_cpp PRIVATE ${YAMLCPP_LIBRARY_DIRS})
  target_link_libraries(plugin_doc_gen_cpp PRIVATE ${YAMLCPP_LIBRARIES})
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(plugin_doc_gen_cpp PRIVATE -O3 -Wall -Wextra -Wpedantic)
endif()

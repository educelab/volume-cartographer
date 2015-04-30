cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(VC_COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(VC_COMMON_INCLUDE_DIR ${VC_COMMON_INCLUDE_DIR} PARENT_SCOPE)

if(TARGET vc_common)
message("vc_common is already defined")
else()
add_library(vc_common STATIC objTester/list.cpp objTester/obj_parser.cpp objTester/objLoader.cpp objTester/objTester.cpp objTester/string_extra.cpp)
endif()

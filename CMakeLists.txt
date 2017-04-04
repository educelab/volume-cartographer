cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(VC VERSION 2.11.0)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Use modern C++ for everything and generate compile_commands database.
set(CMAKE_CXX_STANDARD 14)
set(CXX_STANDARD_REQUIRED on)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

#Choose whether to build statically or dynamically
#option(VC_DYNAMIC_LIBS OFF)

# Choose what to build
option(VC_BUILD_APPS     "Compile VC core programs"    on)
option(VC_BUILD_UTILS    "Compile VC utility programs" on)
option(VC_BUILD_EXAMPLES "Compile VC example programs" on)
option(VC_BUILD_TESTS    "Compile VC test programs"    on)
option(VC_BUILD_DOC      "Compile VC documentation"    off)

# Choose what to install
option(VC_INSTALL_APPS      "Install VC core programs"    on)
option(VC_INSTALL_UTILS     "Install VC utility programs" on)
option(VC_INSTALL_EXAMPLES  "Install VC example programs" off)
option(VC_INSTALL_DOC       "Install VC documentation"    off)

# Look for external dependencies.
include(VCFindDependencies)

# Some helpful constants to be used in subprojects.
include(VCConstants)

# XXX Hack to get ninja output colorized for all source files.
if (CMAKE_GENERATOR MATCHES "Ninja")
    add_compile_options(-fdiagnostics-color=always)
endif()

# Weird bug on macOS where system includes aren't included with the right flag
if (APPLE)
    set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem ")
endif()

if (VC_BUILD_TESTS)
    enable_testing()
endif()

## VC Libraries ##
add_subdirectory(external)

# Enable warnings after 'external' lib.
include(VCWarnings)
add_subdirectory(core)
if (VC_BUILD_TESTS)
    add_subdirectory(testing)
endif()
add_subdirectory(meshing)
add_subdirectory(segmentation)
add_subdirectory(texturing)

#if(VC_DYNAMIC_LIBS)
	#set(BUILD_SHARED_LIBS ON)
#endif()

## VC Core Apps ##
if (VC_BUILD_APPS)
    add_subdirectory(apps)
endif()

## VC Utility apps ##
if (VC_BUILD_UTILS)
    add_subdirectory(utils)
endif()

## VC Example Apps ##
if (VC_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

## VC Documentation
if (VC_BUILD_DOC)
    add_subdirectory(doc)
endif()

# Install to system directories
include(VCInstall)

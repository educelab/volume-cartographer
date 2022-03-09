cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
include(FetchContent)
include(CMakeDependentOption)

set(VC_VERSION 2.23.0)

option(VC_VERSION_DATESTAMP "Append date stamp to version number" off)
if(VC_VERSION_DATESTAMP)
string(TIMESTAMP VC_VERSION_TWEAK "%Y%m%d")
set(VC_VERSION ${VC_VERSION}.${VC_VERSION_TWEAK})
endif()

project(VC VERSION ${VC_VERSION})

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Use modern C++ for everything and generate compile_commands database.
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Choose what to build
option(VC_BUILD_APPS     "Compile VC core programs"    on)
option(VC_BUILD_UTILS    "Compile VC utility programs" on)
option(VC_BUILD_EXAMPLES "Compile VC example programs" on)
option(VC_BUILD_TESTS    "Compile VC test programs"    on)
option(VC_BUILD_PYTHON_BINDINGS "Build Python bindings." off)

# Choose what to install into a CPack package
# Annoyingly, CMake doesn't let you pick components installed by `make install`
option(VC_INSTALL_APPS      "Install VC core programs"    on)
option(VC_INSTALL_LIBS      "Install VC libraries"        on)
option(VC_INSTALL_UTILS     "Install VC utility programs" on)
option(VC_INSTALL_PYTHON_BINDINGS "Install Python bindings." off)

# Some helpful constants to be used in subprojects.
include(VCConstants)

# Look for external dependencies.
include(VCFindDependencies)

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
option(VC_DEVELOPER_WARNINGS "For developers: Enable extensive compiler warnings" OFF)
mark_as_advanced(VC_DEVELOPER_WARNINGS)
if(VC_DEVELOPER_WARNINGS)
    include(VCWarnings)
endif()

add_subdirectory(core)
if (VC_BUILD_TESTS)
    add_subdirectory(testing)
endif()
add_subdirectory(meshing)
add_subdirectory(segmentation)
add_subdirectory(texturing)
add_subdirectory(graph)

# App support library
if(VC_BUILD_APPS OR VC_BUILD_UTILS)
    add_subdirectory(app_support)
endif()

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
find_package(Doxygen OPTIONAL_COMPONENTS dot)
CMAKE_DEPENDENT_OPTION(VC_BUILD_DOCS "Build VC Doxygen documentation" on "DOXYGEN_FOUND" off)
CMAKE_DEPENDENT_OPTION(VC_INSTALL_DOCS "Install VC documentation" off "VC_BUILD_DOCS" off)
if(VC_BUILD_DOCS)
    add_subdirectory(docs)
endif()

if(VC_BUILD_PYTHON_BINDINGS)
    add_subdirectory(python)
endif()

# Setup the config files #
include(VCPackageConfig)
# Install to system directories
include(VCInstall)

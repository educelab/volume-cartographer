cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(VC VERSION 2.8.0)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Use modern C++ for everything
set(CMAKE_CXX_STANDARD 14)
set(CXX_STANDARD_REQUIRED on)

# Global configs
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Choose what to build
option(VC_BUILD_APPS     "Compile VC core programs"    on)
option(VC_BUILD_UTILS    "Compile VC utility programs" on)
option(VC_BUILD_EXAMPLES "Compile VC example programs" on)
option(VC_BUILD_TESTS    "Compile VC test programs"    on)

# Choose what to install
option(VC_INSTALL_APPS      "Install VC core programs"    on)
option(VC_INSTALL_UTILS     "Install VC utility programs" on)
option(VC_INSTALL_EXAMPLES  "Install VC example programs" off)

# Look for external dependencies
include(VCConstants)
include(ConfigDependencies)

# Enable output highlighting if using Ninja
# XXX Hack to get ninja output colorized. I don't think there's an 'all' target
# unless you explicitly make one, otherwise I would use
#
#   target_compile_options(all PUBLIC -fdiagnostics-color=always)
#
if (CMAKE_GENERATOR MATCHES "Ninja")
    add_compile_options(-fdiagnostics-color=always)
endif()

if(VC_BUILD_TESTS)
    enable_testing()
endif()

## VC Libraries ##
# All the targets we wish to install. Filled in by submodules as necessary.
set(installable_targets "")

# The list of include directories to install
set(installable_include_dirs "")

# CMake sub-projects for each major library/component
add_subdirectory(external)
add_subdirectory(core)
if(VC_BUILD_TESTS)
    add_subdirectory(testing)
endif()
add_subdirectory(meshing)
add_subdirectory(segmentation)
add_subdirectory(texturing)

## VC Core Apps ##
if(VC_BUILD_APPS)
    add_subdirectory(apps)
endif()

## VC Utility apps ##
if(VC_BUILD_UTILS)
    add_subdirectory(utils)
endif()

## VC Example Apps ##
if(VC_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

#Install
include(InstallVC)

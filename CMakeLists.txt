cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

project(VC VERSION 2.1.2)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(VC_INSTALL_INCLUDE_PREFIX include/volcart)

# Use modern C++ for everything
set(CMAKE_CXX_STANDARD 11)
set(CXX_STANDARD_REQUIRED on)

# Global configs
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

# Options for external libs
option(VC_USE_ACVD "Use ACVD library" off)
option(VC_USE_BULLET "Use Bullet Physics library" off)
option(VC_USE_LIBIGL "Use libigl" off)

# Use prebuilt libs (built with shell script)?
option(VC_PREBUILT_LIBS "Link against prebuilt dependencies" off)

# Choose what to install
option(VC_INSTALL_CORE "Install VC core apps" on)
option(VC_INSTALL_UTIL "Install VC util apps" on)
option(VC_INSTALL_DEMO "Install VC demo apps" off)

# Look for external dependencies
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

enable_testing()

# VC Libraries
add_subdirectory(external)
add_subdirectory(common)
add_subdirectory(testing)
add_subdirectory(meshing)
add_subdirectory(volumepkg)
add_subdirectory(segmentation)
add_subdirectory(texturing)

# VC Core Apps
add_subdirectory(apps)
add_subdirectory(utils)

# VC Demo Apps
add_subdirectory(examples)

# Install
#include(InstallVC)

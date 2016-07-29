cmake_minimum_required(VERSION 3.6 FATAL_ERROR)

project(VC VERSION 2.1.2)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Options for external libs
option(LINK_ACVD "Link against ACVD library" off)
option(LINK_Bullet "Link against Bullet Physics library" off)

# Use prebuilt libs (built with shell script)?
option(USE_PREBUILT_LIBS "Link against prebuilt dependencies" off)

# Choose what to install
option(INSTALL_VC_CORE "Install VC core apps" on)
option(INSTALL_VC_UTIL "Install VC util apps" on)
option(INSTALL_VC_DEMO "Install VC demo apps" off)

# Look for external dependencies
include(ConfigDependencies)

enable_testing()

# VC Libraries
add_subdirectory(common)
add_subdirectory(meshing)
add_subdirectory(volumepkg)
add_subdirectory(segmentation)
add_subdirectory(texturing)

# VC Core Apps
add_subdirectory(packager)
add_subdirectory(apps)
add_subdirectory(utils)
add_subdirectory(testing)

# VC Demo Apps
add_subdirectory(examples)

# Install
include(InstallVC)

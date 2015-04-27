cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
cmake_policy(SET CMP0048 NEW)
add_definitions("-std=c++11")

project(VC VERSION 1.0.1)


# Prerequisite libraries
set(Boost_USE_STATIC_LIBS ON)

find_package(Qt4 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

IF(APPLE)
	FIND_LIBRARY(SECURITY_LIBRARY Security)
	SET(EXTRA_LIBS ${SECURITY_LIBRARY})
ENDIF(APPLE)

# VC Libraries
add_subdirectory(volumepkg)
include_directories(${VOLUMEPKG_INCLUDE_DIR})

add_subdirectory(meshing)
add_subdirectory(segmentation)
add_subdirectory(examples)

include_directories(${MESHING_INCLUDE_DIR} ${SEGMENTATION_INCLUDE_DIR})

# VC Apps
add_subdirectory(pathgen)
add_subdirectory(reslicer)
add_subdirectory(scalpel)
add_subdirectory(texture)
add_subdirectory(apps)
add_subdirectory(utils)

# Copy packager files. This is a hack.
file(MAKE_DIRECTORY packager)
file(COPY packager/packager.py DESTINATION ${CMAKE_BINARY_DIR}/packager)
file(RENAME ${CMAKE_BINARY_DIR}/packager/packager.py ${CMAKE_BINARY_DIR}/packager/vc_packager)

install(PROGRAMS 
"${CMAKE_BINARY_DIR}/packager/vc_packager" 
"${CMAKE_BINARY_DIR}/pathgen/vc_pathgen" 
"${CMAKE_BINARY_DIR}/scalpel/vc_scalpel" 
"${CMAKE_BINARY_DIR}/apps/vc_segment" 
"${CMAKE_BINARY_DIR}/apps/vc_mesher"
"${CMAKE_BINARY_DIR}/apps/vc_meshEditor" 
"${CMAKE_BINARY_DIR}/texture/vc_texture"
"${CMAKE_BINARY_DIR}/texture/vc_meshnorm"
"${CMAKE_BINARY_DIR}/texture/vc_sliceProjection"
"${CMAKE_BINARY_DIR}/utils/vc_analyze"
"${CMAKE_BINARY_DIR}/utils/vc_conform"
"${CMAKE_BINARY_DIR}/utils/vc_txt2pcd"
DESTINATION bin
COMPONENT Programs)

# Uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

set(CPACK_GENERATOR "ZIP;TGZ")
set(CPACK_COMPONENTS_ALL Programs)
set(CPACK_PACKAGE_NAME "VC")
set(CPACK_PACKAGE_VENDOR "UK VisCenter")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Volume Cartographer")
set(CPACK_PACKAGE_VERSION "1.0.1")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "1")
set(CPACK_MONOLITHIC_INSTALL)
set(CPACK_PACKAGE_INSTALL_DIRECTORY "/usr/local/")

# This must always be last!
include(CPack)
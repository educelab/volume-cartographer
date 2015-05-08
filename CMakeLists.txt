cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
cmake_policy(SET CMP0048 NEW)
add_definitions("-std=c++11")

project(VC VERSION 1.0.1)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# Option for building apps
option(BUILD_OSX_APPS "Build OSX App Bundle" OFF)
option(Boost_USE_STATIC_LIBS "Link against static Boost libs" OFF)

find_package(Qt4 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(ITK REQUIRED)
find_package(OpenGL REQUIRED)

include(${QT_USE_FILE})
include(${ITK_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(${Boost_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIR})
set(VC_REQ_LIBS ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

# OS Specific Changes
IF(APPLE)
	FIND_LIBRARY(SECURITY_LIBRARY Security)
	SET(EXTRA_LIBS ${SECURITY_LIBRARY})
ENDIF(APPLE)

# VC Libraries
add_subdirectory(common)
include_directories(${VC_COMMON_INCLUDE_DIR})

add_subdirectory(meshing)
include_directories(${MESHING_INCLUDE_DIR})

add_subdirectory(volumepkg)
include_directories(${VOLUMEPKG_INCLUDE_DIR})

add_subdirectory(segmentation)
include_directories(${SEGMENTATION_INCLUDE_DIR})

set(VC_LIBS vc_volumepkg vc_meshing vc_segmentation vc_common)

# VC Apps
add_subdirectory(pathgen)
# add_subdirectory(reslicer)
add_subdirectory(scalpel)
add_subdirectory(region)
add_subdirectory(texture)
add_subdirectory(texturing)
add_subdirectory(apps)
add_subdirectory(utils)
add_subdirectory(examples)

# Copy packager files. This is a hack.
file(MAKE_DIRECTORY packager)
file(COPY packager/packager.py DESTINATION ${CMAKE_BINARY_DIR}/bin)
file(RENAME ${CMAKE_BINARY_DIR}/bin/packager.py ${CMAKE_BINARY_DIR}/bin/vc_packager)

install(PROGRAMS 
            "${CMAKE_BINARY_DIR}/bin/vc_packager"
            "${CMAKE_BINARY_DIR}/bin/vc_pathgen"
            "${CMAKE_BINARY_DIR}/bin/vc_scalpel"
            "${CMAKE_BINARY_DIR}/bin/vc_segment"
            "${CMAKE_BINARY_DIR}/bin/vc_region"
            "${CMAKE_BINARY_DIR}/bin/vc_mesher"
            "${CMAKE_BINARY_DIR}/bin/vc_meshEditor"
            "${CMAKE_BINARY_DIR}/bin/vc_texture"
            "${CMAKE_BINARY_DIR}/bin/vc_texture2"
            "${CMAKE_BINARY_DIR}/bin/vc_meshnorm"
            "${CMAKE_BINARY_DIR}/bin/vc_sliceProjection"
            "${CMAKE_BINARY_DIR}/bin/vc_analyze"
            "${CMAKE_BINARY_DIR}/bin/vc_conform"
            "${CMAKE_BINARY_DIR}/bin/vc_txt2pcd"
            "${CMAKE_BINARY_DIR}/bin/vc_metaedit"
            "${CMAKE_BINARY_DIR}/bin/vc_addScale"
        DESTINATION bin
        COMPONENT Programs)

#install(FILES
#            "${CMAKE_BINARY_DIR}/lib/libvc_volumepkg.a"
#            "${CMAKE_BINARY_DIR}/lib/libvc_meshing.a"
#            "${CMAKE_BINARY_DIR}/lib/libvc_segmentation.a"
#        DESTINATION lib/vc
#        COMPONENT Libraries)

#install(FILES
#            "${PROJECT_SOURCE_DIR}/volumepkg/volumepkg.h"
#            "${PROJECT_SOURCE_DIR}/meshing/orderedPCDMesher.h"
#            "${PROJECT_SOURCE_DIR}/segmentation/structureTensorParticleSim.h"
#        DESTINATION include/vc
#        COMPONENT Includes)

# Uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

set(CPACK_GENERATOR "TGZ;ZIP")
set(CPACK_COMPONENTS_ALL Programs)
set(CPACK_PACKAGE_NAME "VC")
set(CPACK_PACKAGE_VENDOR "UK VisCenter")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Volume Cartographer")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_MONOLITHIC_INSTALL)
set(CPACK_PACKAGE_INSTALL_DIRECTORY "/usr/local/")

# This must always be last!
include(CPack)
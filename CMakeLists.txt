cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
cmake_policy(SET CMP0048 NEW)

project(VC VERSION 2.1.2)
set(CMAKE_CXX_STANDARD 11)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# Option for building apps
if(APPLE)
option(BUILD_OSX_APPS "Build OSX App Bundle" OFF)
option(BUILD_DragNDrop "Build OSX DMG Installer" OFF)
endif(APPLE)
option(Boost_USE_STATIC_LIBS "Link against static Boost libs" OFF)
option(VC_PREBUILT_LIBS "Link against prebuilt dependencies" OFF)

# Choose what to install
option(INSTALL_VC_CORE "Install VC core apps" ON)
option(INSTALL_VC_UTIL "Install VC util apps" ON)
option(INSTALL_VC_DEMO "Install VC demo apps" OFF)

IF(VC_PREBUILT_LIBS)
    set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/deps/)
ENDIF(VC_PREBUILT_LIBS)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL PrintSupport)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options iostreams)
find_package(ITK REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenGL REQUIRED)
#find_package(ACVD REQUIRED)

include(${ITK_USE_FILE})
include(${VTK_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(${Boost_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${ACVD_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIR})
set(VC_REQ_LIBS ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ITK_LIBRARIES} ${VTK_LIBRARIES} ${ACVD_LIBRARIES})
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

add_subdirectory(texturing)
include_directories(${TEXTURING_INCLUDE_DIR})

set(VC_LIBS vc_volumepkg vc_meshing vc_segmentation vc_texturing vc_common)

# VC Core Apps
add_subdirectory(packager)
add_subdirectory(pathgen)
add_subdirectory(scalpel)
add_subdirectory(apps)
add_subdirectory(utils)

# VC Demo Apps
#add_subdirectory(region)
add_subdirectory(examples)

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

if(BUILD_DragNDrop)
set(CPACK_GENERATOR "DragNDrop;TGZ;ZIP")
else()
set(CPACK_GENERATOR "TGZ;ZIP")
endif(BUILD_DragNDrop)

if(INSTALL_VC_CORE)
    set(INSTALL_COMPONENTS "${INSTALL_COMPONENTS}" "Programs")
endif(INSTALL_VC_CORE)
if(INSTALL_VC_UTIL)
    set(INSTALL_COMPONENTS "${INSTALL_COMPONENTS}" "Utilities")
endif(INSTALL_VC_UTIL)
if(INSTALL_VC_DEMO)
    set(INSTALL_COMPONENTS "${INSTALL_COMPONENTS}" "Demos")
endif(INSTALL_VC_DEMO)

set(CPACK_COMPONENTS_ALL ${INSTALL_COMPONENTS})
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

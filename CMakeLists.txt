cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
cmake_policy(SET CMP0048 NEW)

project(VC VERSION 2.1.2)
set(CMAKE_CXX_STANDARD 11)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# Export clang compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
add_compile_options(-fdiagnostics-color=always)

# Option for building apps
option(VC_PREBUILT_LIBS "Link against prebuilt dependencies" OFF)

# Choose what to install
option(INSTALL_VC_CORE "Install VC core apps" ON)
option(INSTALL_VC_UTIL "Install VC util apps" ON)
option(INSTALL_VC_DEMO "Install VC demo apps" OFF)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)
IF(VC_PREBUILT_LIBS)
    set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/vc-deps/deps/)
    add_definitions(-DVC_PREBUILT_LIBS)
ENDIF(VC_PREBUILT_LIBS)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL PrintSupport)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options iostreams unit_test_framework)
find_package(ITK REQUIRED)
find_package(VTK 6.1 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(ACVD)
find_package(Bullet)

include(${ITK_USE_FILE})
include(${VTK_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(${Boost_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
set(VC_REQ_LIBS ${PCL_LIBRARIES} pcl_search ${OpenCV_LIBS} ${Boost_LIBRARIES} ${ITK_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

# Conditional Inclusions
if(ACVD_FOUND)
    include_directories(${ACVD_INCLUDE_DIRS})
    set(VC_REQ_LIBS ${VC_REQ_LIBS} ${ACVD_LIBRARIES})
endif(ACVD_FOUND)

if(Bullet_FOUND)
    include_directories(${BULLET_INCLUDE_DIRS})
    set(VC_REQ_LIBS ${VC_REQ_LIBS} ${BULLET_LIBRARIES})
endif(Bullet_FOUND)

# OS Specific Changes
IF(APPLE)
    FIND_LIBRARY(SECURITY_LIBRARY Security)
    SET(EXTRA_LIBS ${SECURITY_LIBRARY})
ENDIF(APPLE)

#for unit testing
enable_testing()

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
add_subdirectory(apps)
add_subdirectory(utils)
add_subdirectory(testing)

# VC Demo Apps
add_subdirectory(examples)

# Uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

if(INSTALL_VC_CORE)
    set(INSTALL_COMPONENTS "${INSTALL_COMPONENTS}" "Programs")
endif(INSTALL_VC_CORE)
if(INSTALL_VC_UTIL)
    set(INSTALL_COMPONENTS "${INSTALL_COMPONENTS}" "Utilities")
endif(INSTALL_VC_UTIL)
if(INSTALL_VC_DEMO)
    set(INSTALL_COMPONENTS "${INSTALL_COMPONENTS}" "Demos")
endif(INSTALL_VC_DEMO)

set(CPACK_GENERATOR "TGZ;ZIP")
set(CPACK_COMPONENTS_ALL ${INSTALL_COMPONENTS})
set(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_VENDOR "UK VisCenter")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Volume Cartographer")
#set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/${README_FILE}")
if(APPLE)
     set(CMAKE_INSTALL_PREFIX "/Applications")
     set(CPACK_GENERATOR "DragNDrop")
     set(CPACK_DMG_FORMAT "UDBZ")
     set(CPACK_DMG_VOLUME_NAME "${PROJECT_NAME}")
     set(CPACK_SYSTEM_NAME "OSX")
     set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}")
     #set(CPACK_PACKAGE_ICON "${CMAKE_SOURCE_DIR}/graphics/vc-logo.icns")
     #set(CPACK_DMG_DS_STORE "${ICONS_DIR}/DMGDSStore")
     #set(CPACK_DMG_BACKGROUND_IMAGE "${ICONS_DIR}/DMGBackground.png")
elseif(UNIX)
    set(CPACK_SYSTEM_NAME "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
endif()

# This must always be last!
include(CPack)

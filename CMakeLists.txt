cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
cmake_policy(SET CMP0048 NEW)
SET(CMAKE_CXX_FLAGS "-std=c++11")

project(VC VERSION 1.0.1)

set(Boost_USE_STATIC_LIBS ON)

add_subdirectory(volumepkg)

add_subdirectory(pathgen)
add_subdirectory(reslicer)
add_subdirectory(scalpel)
add_subdirectory(simulation)
add_subdirectory(mesher)
add_subdirectory(texture)

# Copy packager files. This is a hack.
file(MAKE_DIRECTORY packager)
file(COPY packager/packager.py DESTINATION ${CMAKE_BINARY_DIR}/packager)
file(RENAME ${CMAKE_BINARY_DIR}/packager/packager.py ${CMAKE_BINARY_DIR}/packager/vc_packager)

install(PROGRAMS 
"${CMAKE_BINARY_DIR}/packager/vc_packager" 
"${CMAKE_BINARY_DIR}/pathgen/vc_pathgen" 
"${CMAKE_BINARY_DIR}/scalpel/vc_scalpel" 
"${CMAKE_BINARY_DIR}/simulation/vc_simulation" 
"${CMAKE_BINARY_DIR}/mesher/vc_mesher" 
"${CMAKE_BINARY_DIR}/texture/vc_texture"
"${CMAKE_BINARY_DIR}/texture/vc_meshnorm"
"${CMAKE_BINARY_DIR}/texture/vc_sliceProjection"
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
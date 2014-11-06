cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(SET CMP0048 NEW)
SET(CMAKE_CXX_FLAGS "-std=c++11")

project(VC VERSION 1.0)

add_subdirectory(volumepkg)

add_subdirectory(pathgen)
add_subdirectory(reslicer)
add_subdirectory(scalpel)
add_subdirectory(simulation)
add_subdirectory(texture)

install(PROGRAMS "${CMAKE_BINARY_DIR}/pathgen/vc_pathgen" "${CMAKE_BINARY_DIR}/scalpel/vc_scalpel" "${CMAKE_BINARY_DIR}/simulation/vc_simulation" "${CMAKE_BINARY_DIR}/texture/vc_texture" DESTINATION bin)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
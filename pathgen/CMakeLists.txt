cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)

add_subdirectory(./../volumepkg volumepkg)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${VOLUMEPKG_INCLUDE_DIR})

add_executable(pathgen pathGen.cpp timeUtils.cpp)

target_link_libraries(pathgen volumepkg ${OpenCV_LIBS})

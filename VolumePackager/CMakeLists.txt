cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(volumepkg)

find_package(OpenCV REQUIRED)

add_executable(volumepkg volumepkg.cpp volumepkgcfg.cpp)

target_link_libraries(volumepkg ${OpenCV_LIBS})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(./../volumepkg volumepkg)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${VOLUMEPKG_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(vc_simulation simulation.cpp)

target_link_libraries(vc_simulation ${PCL_LIBRARIES})
target_link_libraries(vc_simulation ${OpenCV_LIBS})
target_link_libraries(vc_simulation volumepkg)

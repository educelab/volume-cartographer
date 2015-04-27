cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(GenMesh)

find_package(ITK REQUIRED)
#find_package(PCL 1.7 REQUIRED)
#find_package(OpenCV REQUIRED)

include(${ITK_USE_FILE})
#add_subdirectory(./../volumepkg volumepkg)
#include_directories(${VOLUMEPKG_INCLUDE_DIR})
#include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable(GenMesh main.cpp UPointMapping.cpp)

#target_link_libraries(GenMesh ITKCommon)
target_link_libraries(GenMesh ${ITK_LIBRARIES})
target_link_libraries(GenMesh ${PCL_LIBRARIES})
target_link_libraries(GenMesh ${OpenCV_LIBS})
target_link_libraries(GenMesh volumepkg)

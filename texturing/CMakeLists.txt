cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

include(${ITK_USE_FILE})

add_executable(GenMesh main.cpp UPointMapping.cpp plyHelper.cpp)

target_link_libraries(GenMesh ${ITK_LIBRARIES})
target_link_libraries(GenMesh ${PCL_LIBRARIES})
target_link_libraries(GenMesh ${OpenCV_LIBS})
target_link_libraries(GenMesh volumepkg)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(vc_scalpel scalpel.cpp)

target_link_libraries(vc_scalpel ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries(vc_scalpel ${OpenCV_LIBS})
target_link_libraries(vc_scalpel volumepkg)
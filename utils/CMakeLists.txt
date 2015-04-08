cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(vc_analyze analyze.cpp)
add_executable(vc_conform conform.cpp)

target_link_libraries(vc_analyze ${OpenCV_LIBS})
target_link_libraries(vc_conform ${OpenCV_LIBS})

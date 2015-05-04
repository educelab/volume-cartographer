cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(vc_scalpel scalpel.cpp)

target_link_libraries(vc_scalpel ${VC_LIBS} ${VC_REQ_LIBS})
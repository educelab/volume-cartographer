cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

add_executable(vc_texture2 layering.cpp plyHelper.cpp meshUtils.cpp)

target_link_libraries(vc_texture2 ${ITK_LIBRARIES})
target_link_libraries(vc_texture2 ${VC_LIBS} ${VC_REQ_LIBS})

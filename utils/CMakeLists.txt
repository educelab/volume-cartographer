cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(vc_analyze analyze.cpp)
add_executable(vc_conform conform.cpp)
add_executable(vc_txt2pcd txt2pcd.cpp)
add_executable(vc_addScale addScale.cpp)
add_executable(vc_zxy2xyz zxy2xyz.cpp)

target_link_libraries(vc_analyze ${OpenCV_LIBS})
target_link_libraries(vc_conform ${OpenCV_LIBS})
target_link_libraries(vc_txt2pcd ${PCL_LIBRARIES})
target_link_libraries(vc_addScale ${VC_LIBS} ${VC_REQ_LIBS})
target_link_libraries(vc_zxy2xyz ${PCL_LIBRARIES})

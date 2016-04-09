cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(vc_txt2pcd txt2pcd.cpp)
add_executable(vc_addScale addScale.cpp)
add_executable(vc_zxy2xyz zxy2xyz.cpp)
add_executable(vc_area surfarea.cpp)
add_executable(vc_splitcloud splitcloud.cpp)
add_executable(vc_invertcloud invertCloud.cpp)

target_link_libraries(vc_txt2pcd ${PCL_LIBRARIES})
target_link_libraries(vc_addScale ${VC_LIBS} ${VC_REQ_LIBS})
target_link_libraries(vc_zxy2xyz ${PCL_LIBRARIES})
target_link_libraries(vc_area ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})
target_link_libraries(vc_splitcloud ${PCL_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(vc_invertcloud ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_seg2printable seg2Printable.cpp)
target_link_libraries(vc_seg2printable ${VC_LIBS} ${VC_REQ_LIBS})

add_executable(vc_csv2uv csv2uv.cpp)
target_link_libraries(vc_csv2uv ${VC_LIBS} ${VC_REQ_LIBS} ${VTK_LIBRARIES})

# Install targets
install(TARGETS vc_txt2pcd vc_zxy2xyz vc_area vc_seg2printable vc_splitcloud
        RUNTIME DESTINATION bin COMPONENT Utilities)

install(TARGETS vc_addScale
        RUNTIME DESTINATION bin COMPONENT Demos)

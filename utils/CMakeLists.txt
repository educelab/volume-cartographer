add_executable(vc_txt2pcd src/txt2pcd.cpp)
target_link_libraries(vc_txt2pcd pcl_common pcl_io)

add_executable(vc_addScale src/addScale.cpp)
target_link_libraries(vc_addScale vc_volumepkg vc_common)

add_executable(vc_zxy2xyz src/zxy2xyz.cpp)
target_link_libraries(vc_zxy2xyz pcl_io pcl_common)

add_executable(vc_area src/surfarea.cpp)
target_link_libraries(vc_area vc_volumepkg vc_common vc_meshing)

add_executable(vc_splitcloud src/splitcloud.cpp)
target_link_libraries(vc_splitcloud
    Boost::filesystem
    pcl_io
    pcl_common
)

add_executable(vc_invertcloud src/invertCloud.cpp)
target_link_libraries(vc_invertcloud vc_volumepkg vc_common)

add_executable(vc_seg2printable seg2Printable.cpp)
target_link_libraries(vc_seg2printable vc_common vc_volumepkg vc_meshing)

add_executable(vc_mappingExtractor mappingExtractor.cpp)
target_link_libraries(vc_mappingExtractor vc_volumepkg vc_common)

add_executable(vc_csv2uv csv2uv.cpp)
target_link_libraries(vc_csv2uv vc_common vc_meshing)

add_executable(pcl_convert_pcd_ascii_binary src/convert_pcd_ascii_binary.cpp)
target_link_libraries(pcl_convert_pcd_ascii_binary pcl_common pcl_io)

# Install targets
install(TARGETS vc_txt2pcd vc_zxy2xyz vc_area vc_seg2printable vc_splitcloud
        RUNTIME DESTINATION bin COMPONENT Utilities)

install(TARGETS vc_addScale
        RUNTIME DESTINATION bin COMPONENT Demos)

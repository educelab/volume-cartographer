# vc_text2pcd
add_executable(vc_txt2pcd src/txt2pcd.cpp)
target_include_directories(vc_txt2pcd PUBLIC
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_IO_INCLUDE_DIRS}
)
target_link_libraries(vc_txt2pcd ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

# vc_addScale
add_executable(vc_addScale src/addScale.cpp)
target_link_libraries(vc_addScale vc_volumepkg vc_common)

# vc_zxy2xyz
add_executable(vc_zxy2xyz src/zxy2xyz.cpp)
target_include_directories(vc_zxy2xyz PUBLIC
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_IO_INCLUDE_DIRS}
)
target_link_libraries(vc_zxy2xyz ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES})

# vc_area
add_executable(vc_area src/surfarea.cpp)
target_link_libraries(vc_area vc_volumepkg vc_common vc_meshing)

# vc_splitcloud
add_executable(vc_splitcloud src/splitcloud.cpp)
target_include_directories(vc_splitcloud PUBLIC
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_IO_INCLUDE_DIRS}
)
target_link_libraries(vc_splitcloud
    Boost::filesystem
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)

# vc_invertcloud
add_executable(vc_invertcloud src/invertCloud.cpp)
target_include_directories(vc_invertcloud PUBLIC
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_IO_INCLUDE_DIRS}
)
target_link_libraries(vc_invertcloud
    vc_volumepkg
    vc_common
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)

# vc_seg2printable
add_executable(vc_seg2printable src/seg2Printable.cpp)
target_link_libraries(vc_seg2printable
    vc_common
    vc_volumepkg
    vc_meshing
    vc_texturing
)

# vc_mappingExtractor
add_executable(vc_mappingExtractor src/mappingExtractor.cpp)
target_link_libraries(vc_mappingExtractor vc_volumepkg vc_common)

# vc_csv2uv
add_executable(vc_csv2uv src/csv2uv.cpp)
target_link_libraries(vc_csv2uv vc_common vc_meshing vtkIOPLY)

# pcl_convert_pcd_ascii_binary
add_executable(pcl_convert_pcd_ascii_binary src/convert_pcd_ascii_binary.cpp)
target_include_directories(pcl_convert_pcd_ascii_binary PUBLIC
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_IO_INCLUDE_DIRS}
)
target_link_libraries(pcl_convert_pcd_ascii_binary
    ${PCL_COMMON_LIBRARIES}
    ${PCL_IO_LIBRARIES}
)

# Install targets
install(TARGETS vc_area
        RUNTIME DESTINATION bin COMPONENT Utilities)

install(TARGETS vc_addScale
        RUNTIME DESTINATION bin COMPONENT Demos)

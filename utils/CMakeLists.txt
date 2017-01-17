# Install targets list for Utilities
set(VC_INSTALL_UTILS "")

# vc_addScale
add_executable(vc_addScale src/AddScale.cpp)
target_link_libraries(vc_addScale VC::core)
list(APPEND VC_INSTALL_UTILS vc_addScale)

# vc_area
add_executable(vc_area src/SurfArea.cpp)
target_link_libraries(vc_area VC::core VC::meshing Boost::filesystem)
list(APPEND VC_INSTALL_UTILS vc_area)

# vc_invertcloud
add_executable(vc_invertcloud src/InvertCloud.cpp)
target_link_libraries(vc_invertcloud VC::core Boost::filesystem)

# vc_mappingExtractor
add_executable(vc_mappingExtractor src/MappingExtractor.cpp)
target_link_libraries(vc_mappingExtractor VC::core)

# vc_csv2uv
add_executable(vc_csv2uv src/CSV2UV.cpp)
target_link_libraries(vc_csv2uv VC::core VC::meshing vtkIOPLY)

# vc_convert_pcd_to_ps
if(PCL_FOUND)
    add_executable(vc_convert_pcd_to_ps src/PointCloudToPointSet.cpp)
    target_include_directories(vc_convert_pcd_to_ps
        PUBLIC
            pcl_common
            pcl_io
    )
    target_link_libraries(vc_convert_pcd_to_ps
        VC::core
        VC::meshing
        Boost::filesystem
        pcl_common
        pcl_io
    )
    list(APPEND VC_INSTALL_UTILS vc_convert_pcd_to_ps)
endif()

# Install targets
install(
    TARGETS ${VC_INSTALL_UTILS}
    RUNTIME DESTINATION bin
    COMPONENT Utilities
)

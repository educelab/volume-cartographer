# Install targets list for Utilities
set(VC_INSTALL_UTILS "")

# vc_addScale
add_executable(vc_addScale src/addScale.cpp)
target_link_libraries(vc_addScale vc_volumepkg vc_common)
list(APPEND VC_INSTALL_UTILS vc_addScale)

# vc_area
add_executable(vc_area src/surfarea.cpp)
target_link_libraries(vc_area
    vc_volumepkg
    vc_common
    vc_meshing
    Boost::filesystem
)
list(APPEND VC_INSTALL_UTILS vc_area)

# vc_invertcloud
add_executable(vc_invertcloud src/invertCloud.cpp)
target_link_libraries(vc_invertcloud
    vc_volumepkg
    vc_common
    Boost::filesystem
)

# vc_mappingExtractor
add_executable(vc_mappingExtractor src/mappingExtractor.cpp)
target_link_libraries(vc_mappingExtractor vc_volumepkg vc_common)

# vc_csv2uv
add_executable(vc_csv2uv src/csv2uv.cpp)
target_link_libraries(vc_csv2uv vc_common vc_meshing vtkIOPLY)

# vc_convert_pcd_to_ps
if(PCL_FOUND)
    add_executable(vc_convert_pcd_to_ps src/PointCloudToPointSet.cpp)
    target_include_directories(vc_convert_pcd_to_ps
        PUBLIC
            ${PCL_COMMON_INCLUDE_DIRS}
            ${PCL_IO_INCLUDE_DIRS}
    )
    target_link_libraries(vc_convert_pcd_to_ps
        vc_common
        vc_volumepkg
        Boost::filesystem
        ${PCL_COMMON_LIBRARIES}
        ${PCL_IO_LIBRARIES}
    )
    list(APPEND VC_INSTALL_UTILS vc_convert_pcd_to_ps)
endif()

# Install targets
install(TARGETS ${VC_INSTALL_UTILS}
        RUNTIME DESTINATION bin
        COMPONENT Utilities)

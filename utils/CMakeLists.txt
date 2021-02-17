# vc_area
add_executable(vc_area src/SurfArea.cpp)
target_link_libraries(vc_area VC::core VC::meshing ${VC_FS_LIB} vtkFiltersCore)
list(APPEND utils_install_list vc_area)

# vc_invertcloud
add_executable(vc_invertcloud src/InvertCloud.cpp)
target_link_libraries(vc_invertcloud VC::core ${VC_FS_LIB})

# vc_mappingExtractor
add_executable(vc_mappingExtractor src/MappingExtractor.cpp)
target_link_libraries(vc_mappingExtractor VC::core)

# vc_csv2uv
add_executable(vc_csv2uv src/CSV2UV.cpp)
target_link_libraries(vc_csv2uv VC::core VC::meshing vtkIOPLY vtkFiltersCore)

# vc_volpkg_upgrade
add_executable(vc_volpkg_upgrade src/VolpkgUpgrade.cpp)
target_link_libraries(vc_volpkg_upgrade VC::core)
list(APPEND utils_install_list vc_volpkg_upgrade)

# vc_flip_mesh
add_executable(vc_flip_mesh src/FlipMesh.cpp)
target_link_libraries(vc_flip_mesh VC::core ${VC_FS_LIB} Boost::program_options)

# vc_flip_uv
add_executable(vc_flip_uv src/FlipUV.cpp)
target_link_libraries(vc_flip_uv VC::core ${VC_FS_LIB} Boost::program_options)

# vc_retexture_mesh
add_executable(vc_retexture_mesh src/RetextureMesh.cpp)
target_link_libraries(vc_retexture_mesh VC::core ${VC_FS_LIB} Boost::program_options)
list(APPEND utils_install_list vc_retexture_mesh)

# vc_ppm_to_pointset
add_executable(vc_ppm_to_pointset src/PPMToPointSet.cpp)
target_link_libraries(vc_ppm_to_pointset VC::core ${VC_FS_LIB} Boost::program_options)

# vc_canny_segment
add_executable(vc_canny_segment src/CannySegment.cpp)
target_link_libraries(vc_canny_segment VC::app_support VC::core ${VC_FS_LIB} Boost::program_options opencv_core)
list(APPEND utils_install_list vc_canny_segment)

# vc_uv2mesh
add_executable(vc_uv2mesh src/UVMaptoFlatMesh.cpp)
target_link_libraries(vc_uv2mesh
    VC::core
    VC::meshing
    ${VC_FS_LIB}
    Boost::program_options
)

# vc_volume_bump
add_executable(vc_volume_bump src/VolumeBump.cpp)
target_link_libraries(vc_volume_bump
    VC::app_support
    VC::core
    ${VC_FS_LIB}
    Boost::program_options
    opencv_core
    opencv_imgcodecs
)

# vc_transform_mesh
add_executable(vc_transform_mesh src/TransformMesh.cpp)
target_link_libraries(vc_transform_mesh
    VC::core
    ${VC_FS_LIB}
    Boost::program_options
    ITKIOTransformBase
    ITKIOTransformInsightLegacy
)
list(APPEND utils_install_list vc_transform_mesh)

# vc_add_alignment_markers
add_executable(vc_add_alignment_markers src/AlignmentMarkers.cpp)
target_link_libraries(vc_add_alignment_markers
    VC::core
    VC::texturing
    ${VC_FS_LIB}
    Boost::program_options
)

# vc_convert_pointset
add_executable(vc_convert_pointset src/ConvertPointSet.cpp)
target_link_libraries(vc_convert_pointset
    VC::app_support
    VC::core
    VC::texturing
    ${VC_FS_LIB}
    Boost::program_options
)
list(APPEND utils_install_list vc_convert_pointset)

# vc_merge_pointsets
add_executable(vc_merge_pointset src/MergePointSets.cpp)
target_link_libraries(vc_merge_pointset
    VC::core
    ${VC_FS_LIB}
    Boost::program_options
)
list(APPEND utils_install_list vc_merge_pointset)

# vc_convert_mask
add_executable(vc_convert_mask src/ConvertMask.cpp)
target_link_libraries(vc_convert_mask
    VC::core
    VC::app_support
    opencv_core
    opencv_imgcodecs
    ${VC_FS_LIB}
    Boost::program_options
)

# vc_seg_to_pointmask
add_executable(vc_seg_to_pointmask src/SegToPointMask.cpp)
target_link_libraries(vc_seg_to_pointmask
    VC::core
    VC::segmentation
    VC::app_support
    opencv_core
    ${VC_FS_LIB}
    Boost::program_options
)

# vc_visualize_ppm
add_executable(vc_visualize_ppm src/VisualizePPM.cpp)
target_link_libraries(vc_visualize_ppm
    VC::core
    VC::app_support
    ${VC_FS_LIB}
    Boost::program_options
)

# vc_image_stats
add_executable(vc_image_stats src/ImageStatistics.cpp)
target_link_libraries(vc_image_stats
    VC::core
    VC::app_support
    opencv_core
    opencv_imgcodecs
    ${VC_FS_LIB}
    Boost::program_options
)

# vc_flatten_mesh
add_executable(vc_flatten_mesh src/FlattenMesh.cpp)
target_link_libraries(vc_flatten_mesh
    VC::core
    VC::texturing
    ${VC_FS_LIB}
    Boost::program_options
)
list(APPEND utils_install_list vc_flatten_mesh)

# Install targets
if(VC_INSTALL_UTILS)
install(
    TARGETS ${utils_install_list}
    RUNTIME DESTINATION bin
    COMPONENT Utilities
)
endif()

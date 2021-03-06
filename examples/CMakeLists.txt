if (VC_USE_ACVD OR VC_USE_OPTIONAL)
    add_executable(vc_acvd_example src/ACVDExample.cpp)
    target_link_libraries(vc_acvd_example
        VC::core
        VC::meshing
    )
endif()

#LANDMARKS TESTING
add_executable(vc_landmarks_example src/CreateLandmarks.cpp)
target_link_libraries(vc_landmarks_example VC::core)

add_executable(vc_volpkg_example src/Volpkg.cpp)
target_link_libraries(vc_volpkg_example VC::core)

add_executable(vc_uvmap_example src/UVMapExample.cpp)
target_link_libraries(vc_uvmap_example VC::core)

add_executable(vc_texture_flattened src/TextureFlattened.cpp)
target_link_libraries(vc_texture_flattened
    VC::core
    VC::meshing
    VC::texturing
    vtkIOPLY
)

add_executable(vc_texture_example src/TextureExample.cpp)
target_link_libraries(vc_texture_example VC::core VC::meshing VC::texturing)

add_executable(vc_render_with_scale_example src/ScaleMarkerGeneratorExample.cpp)
target_link_libraries(vc_render_with_scale_example VC::core VC::texturing opencv_imgcodecs)

add_executable(vc_smooth_normals_example src/SmoothNormalsExample.cpp)
target_link_libraries(vc_smooth_normals_example VC::core VC::meshing)

add_executable(vc_scale_mesh_example src/ScaleMeshExample.cpp)
target_link_libraries(vc_scale_mesh_example VC::core VC::meshing)

if(VC_BUILD_TESTS)
add_executable(vc_bad_clang_tidy src/BadClangTidy.cpp)
target_link_libraries(vc_bad_clang_tidy vc_testing)
endif(VC_BUILD_TESTS)

add_executable(vc_raytrace_example src/RayTraceExample.cpp)
target_link_libraries(vc_raytrace_example VC::core VC::meshing)

add_executable(vc_bad_clang_format src/BadClangFormat2.cpp)

add_executable(vc_itk2vtk_example src/ITK2VTKExample.cpp)
target_link_libraries(vc_itk2vtk_example
    VC::core
    VC::meshing
    VC::testing
)

if (VC_BUILD_EXPERIMENTAL)
    add_executable(vc_cloth_example src/ClothExample.cpp)
    target_link_libraries(vc_cloth_example
        VC::core
        VC::experimental
        VC::texturing
        VC::meshing
    )
endif()

if (VC_USE_ACVD OR VC_USE_OPTIONAL)
    add_executable(vc_abf_example src/ABFExample.cpp)
    target_link_libraries(vc_abf_example VC::core VC::texturing)
endif()

add_executable(vc_structure_tensor_client src/StructureTensorClient.cpp)
target_link_libraries(vc_structure_tensor_client VC::core opencv_highgui)

add_executable(vc_reslice_analysis src/ResliceAnalysis.cpp)
target_link_libraries(vc_reslice_analysis VC::core opencv_highgui)

add_executable(vc_ordered_resample_example src/OrderedResamplingExample.cpp)
target_link_libraries(vc_ordered_resample_example VC::core VC::meshing)

add_executable(vc_obj_writer_example src/OBJWriterExample.cpp)
target_link_libraries(vc_obj_writer_example VC::core)

add_executable(vc_lorem_ipsum_error src/LoremIpsumError.cpp)
target_link_libraries(vc_lorem_ipsum_error VC::core)

if(VC_USE_VCG OR VC_USE_OPTIONAL)
    add_executable(vc_quadric_edge_collapse_decimation_example
        src/QuadricEdgeCollapseDecimationExample.cpp
    )
    target_link_libraries(vc_quadric_edge_collapse_decimation_example
        VC::meshing
    )
endif()

add_executable(vc_orderedPointSetMesher_example
    src/OrderedPointSetMesherExample.cpp
)
target_link_libraries(vc_orderedPointSetMesher_example VC::meshing)

add_executable(vc_obj_reader_example src/OBJReaderExample.cpp)
target_link_libraries(vc_obj_reader_example VC::core)

add_executable(vc_stps_example src/STPSExample.cpp)
target_link_libraries(vc_stps_example VC::core VC::meshing VC::segmentation)

add_executable(vc_spiral_example src/SpiralGenerator.cpp)
target_link_libraries(vc_spiral_example VC::core)

add_executable(vc_scale_marker_generator_example src/ScaleMarkerGeneratorExample.cpp)
target_link_libraries(vc_scale_marker_generator_example VC::core VC::texturing opencv_imgcodecs)

add_executable(vc_signals_example src/SignalsExample.cpp)
target_link_libraries(vc_signals_example VC::core)
if (VC_USE_ACVD)
    add_executable(vc_acvd_example src/ACVDExample.cpp)
    target_link_libraries(vc_acvd_example
        vc_core
        vc_meshing
        acvd
    )
endif()

add_executable(vc_volpkg_example src/Volpkg.cpp)
target_link_libraries(vc_volpkg_example vc_core)

add_executable(vc_uvmap_example src/UVMapExample.cpp)
target_link_libraries(vc_uvmap_example vc_core)

add_executable(vc_texture_flattened src/TextureFlattened.cpp)
target_link_libraries(vc_texture_flattened
    vc_core
    vc_meshing
    vc_texturing
    vtkIOPLY
)

add_executable(vc_texture_example src/TextureExample.cpp)
target_link_libraries(vc_texture_example vc_core vc_texturing)

add_executable(vc_smooth_normals_example src/SmoothNormalsExample.cpp)
target_link_libraries(vc_smooth_normals_example vc_core vc_meshing)

add_executable(vc_scale_mesh_example src/ScaleMeshExample.cpp)
target_link_libraries(vc_scale_mesh_example vc_core vc_meshing ITKIOMesh)


add_executable(vc_raytrace_example src/RayTraceExample.cpp)
target_link_libraries(vc_raytrace_example vc_core vc_meshing)

if (VC_USE_LIBIGL)
    add_executable(vc_lscm_example src/LSCMExample.cpp)
    target_link_libraries(vc_lscm_example vc_core vc_texturing)
endif()

add_executable(vc_ITK2VTK_example src/ITK2VTKExample.cpp)
target_link_libraries(vc_ITK2VTK_example
    vc_core
    vc_meshing
    vc_testing
    ITKIOMesh
)

if (VC_USE_BULLET)
    add_executable(vc_cloth_example src/ClothExample.cpp)
    target_link_libraries(vc_cloth_example vc_core vc_texturing vc_meshing ${BULLET_LIBRARIES})
endif()

if (VC_USE_ACVD)
    add_executable(vc_abf_texturing src/ABFTexturing.cpp)
    target_link_libraries(vc_abf_texturing
        vc_core
        vc_meshing
        vc_texturing
        Boost::filesystem
    )

    add_executable(vc_abf_example src/ABFExample.cpp)
    target_link_libraries(vc_abf_example vc_core vc_texturing)
endif()

add_executable(vc_structure_tensor_client src/StructureTensorClient.cpp)
target_link_libraries(vc_structure_tensor_client vc_core opencv_highgui)

add_executable(vc_reslice_analysis src/ResliceAnalysis.cpp)
target_link_libraries(vc_reslice_analysis vc_core opencv_highgui)

add_executable(vc_ordered_resample_example src/OrderedResamplingExample.cpp)
target_link_libraries(vc_ordered_resample_example vc_core vc_meshing)

add_executable(vc_obj_writer_example src/OBJWriterExample.cpp)
target_link_libraries(vc_obj_writer_example vc_core)

# Skip NeuralNetwork train example

add_executable(vc_lorem_ipsum_error src/LoremIpsumError.cpp)
target_link_libraries(vc_lorem_ipsum_error vc_core)

if(VC_USE_VCG)
    add_executable(vc_quadric_edge_collapse_decimation_example src/QuadricEdgeCollapseDecimationExample.cpp)
    target_link_libraries(vc_quadric_edge_collapse_decimation_example vc_meshing)
endif()

add_executable(vc_asantest src/ASANTest.cpp)
add_sanitizers(vc_asantest)

add_executable(vc_orderedPointSetMesher_example src/OrderedPointSetMesherExample.cpp)
target_link_libraries(vc_orderedPointSetMesher_example vc_meshing)

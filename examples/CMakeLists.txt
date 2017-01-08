if (VC_USE_ACVD)
    add_executable(vc_acvd_example src/ACVDExample.cpp)
    target_link_libraries(vc_acvd_example
        VC::core
        VC::meshing
        acvd
    )
endif()

add_executable(vc_volpkg_example src/volpkg.cpp)
target_link_libraries(vc_volpkg_example VC::core)

add_executable(vc_uvmap_example src/uvMapExample.cpp)
target_link_libraries(vc_uvmap_example VC::core)

add_executable(vc_texture_flattened src/textureFlattened.cpp)
target_link_libraries(vc_texture_flattened
    VC::core
    VC::meshing
    VC::texturing
    vtkIOPLY
)

add_executable(vc_texture_example src/textureExample.cpp)
target_link_libraries(vc_texture_example VC::core VC::texturing)

add_executable(vc_smooth_normals_example src/smoothNormalsExample.cpp)
target_link_libraries(vc_smooth_normals_example VC::core VC::meshing)

add_executable(vc_scale_mesh_example src/scaleMeshExample.cpp)
target_link_libraries(vc_scale_mesh_example VC::core VC::meshing ITKIOMesh)


add_executable(vc_raytrace_example src/rayTraceExample.cpp)
target_link_libraries(vc_raytrace_example VC::core VC::meshing)

if (VC_USE_LIBIGL)
    add_executable(vc_lscm_example src/lscmExample.cpp)
    target_link_libraries(vc_lscm_example VC::core VC::texturing)
endif()

add_executable(vc_itk2vtk_example src/itk2vtkExample.cpp)
target_link_libraries(vc_itk2vtk_example
    VC::core
    VC::meshing
    VC::testing
    ITKIOMesh
)

if (VC_USE_BULLET)
    add_executable(vc_cloth_example src/clothExample.cpp)
    target_link_libraries(vc_cloth_example
        VC::core
        VC::texturing
        VC::meshing
        ${BULLET_LIBRARIES}
    )
endif()

if (VC_USE_ACVD)
    add_executable(vc_abf_texturing src/abfTexturing.cpp)
    target_link_libraries(vc_abf_texturing
        VC::core
        VC::meshing
        VC::texturing
        Boost::filesystem
    )

    add_executable(vc_abf_example src/abfExample.cpp)
    target_link_libraries(vc_abf_example VC::core VC::texturing)
endif()

add_executable(vc_structure_tensor_client src/StructureTensorClient.cpp)
target_link_libraries(vc_structure_tensor_client VC::core opencv_highgui)

add_executable(vc_reslice_analysis src/ResliceAnalysis.cpp)
target_link_libraries(vc_reslice_analysis VC::core opencv_highgui)

add_executable(vc_ordered_resample_example src/OrderedResamplingExample.cpp)
target_link_libraries(vc_ordered_resample_example VC::core VC::meshing)

add_executable(vc_obj_writer_example src/ObjWriterExample.cpp)
target_link_libraries(vc_obj_writer_example VC::core)

# Skip NeuralNetwork train example

add_executable(vc_lorem_ipsum_error src/LoremIpsumError.cpp)
target_link_libraries(vc_lorem_ipsum_error VC::core)

if(VC_USE_VCG)
    add_executable(vc_quadric_edge_collapse_decimation_example
        src/QuadricEdgeCollapseDecimationExample.cpp
    )
    target_link_libraries(vc_quadric_edge_collapse_decimation_example
        VC::meshing
    )
endif()

add_executable(vc_asantest src/ASANTest.cpp)
add_sanitizers(vc_asantest)

add_executable(vc_orderedPointSetMesher_example
    src/OrderedPointSetMesherExample.cpp
)
target_link_libraries(vc_orderedPointSetMesher_example VC::meshing)

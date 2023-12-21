add_executable(vc_abf_example src/ABFExample.cpp)
target_link_libraries(vc_abf_example VC::core VC::texturing)

add_executable(vc_acvd_example src/ACVDExample.cpp)
target_link_libraries(vc_acvd_example VC::core VC::meshing)

add_executable(vc_itk2vtk_example src/ITK2VTKExample.cpp)
target_link_libraries(vc_itk2vtk_example VC::core VC::meshing)

add_executable(vc_obj_writer_example src/OBJWriterExample.cpp)
target_link_libraries(vc_obj_writer_example VC::core)

add_executable(vc_ordered_mesher_example src/OrderedPointSetMesherExample.cpp)
target_link_libraries(vc_ordered_mesher_example VC::meshing)

add_executable(vc_ordered_resample_example src/OrderedResamplingExample.cpp)
target_link_libraries(vc_ordered_resample_example VC::core VC::meshing)

add_executable(vc_render_graphs_example src/RenderGraphsExample.cpp)
target_link_libraries(vc_render_graphs_example VC::graph smgl::smgl)

add_executable(vc_scale_mesh_example src/ScaleMeshExample.cpp)
target_link_libraries(vc_scale_mesh_example VC::core VC::meshing)

add_executable(vc_signals_example src/SignalsExample.cpp)
target_link_libraries(vc_signals_example VC::core)

add_executable(vc_smooth_normals_example src/SmoothNormalsExample.cpp)
target_link_libraries(vc_smooth_normals_example VC::core VC::meshing)

add_executable(vc_apply_transform_example src/ApplyTransformsExample.cpp)
target_link_libraries(vc_apply_transform_example VC::core VC::texturing)
project(libvc_meshing VERSION 0.1 LANGUAGES CXX)
set(target vc_meshing)

set(srcs
    src/deepCopy.cpp
    src/itk2vtk.cpp
    src/orderedPCDMesher.cpp
    src/rayTrace.cpp
    src/resamplePointCloud.cpp
    src/scaleMesh.cpp
    src/smoothNormals.cpp
    src/CalculateNormals.cpp
    src/OrderedResampling.cpp
)
set(deps
    vc_common
)
set(defs "")

# Conditional compilation of ACVD and Bullet stuff
if (VC_USE_ACVD)
    list(APPEND srcs src/ACVD.cpp)
    list(APPEND deps acvd)
    list(APPEND defs VC_USE_ACVD)
endif()
if (VC_USE_BULLET)
    list(APPEND srcs src/itk2bullet.cpp)
    list(APPEND deps bullet)
    list(APPEND defs VC_USE_BULLET)
endif()

add_library(${target} ${srcs})
target_compile_definitions(${target} PUBLIC ${defs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC
        ${deps} 
    PRIVATE
        ITKTransform
        vtkFiltersGeneral
        vtkFiltersCore
        ${PCL_SEARCH_LIBRARIES}
        ${PCL_KDTREE_LIBRARIES}
        ${PCL_SURFACE_LIBRARIES}
        ${PCL_FILTERS_LIBRARIES}
        ${PCL_IO_LIBRARIES}
)

install(TARGETS ${target}
    ARCHIVE DESTINATION lib COMPONENT Libraries
    LIBRARY DESTINATION lib COMPONENT Libraries
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX} COMPONENT Includes)
export(TARGETS ${target} FILE VCMeshing.cmake)

### Testing ###
if(VC_BUILD_TESTS)

# Set source files
set(test_srcs
    test/CalculateNormalsTest.cpp
    test/OrderedResamplingTest.cpp
    test/itk2vtkTest.cpp
    test/orderedPCDMesherTest.cpp
    test/rayTraceTest.cpp
    test/resamplePointCloudTest.cpp
    test/scaleMeshTest.cpp
    test/smoothNormalsTest.cpp
)

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        vc_common
        vc_meshing
        vc_testing
        Boost::unit_test_framework
    )
    add_test(
        NAME ${testname}
        WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
        COMMAND ${testname}
    )
endforeach()

# Copy testing resources to bin/ directory
set(ITK_VTK_TEST_RES
    test/res/ITKPlaneMeshConvertedToVTK.ply 
    test/res/ITKCubeMeshConvertedToVTK.ply
    test/res/ITKArchMeshConvertedToVTK.ply
    test/res/ITKSphereMeshConvertedToVTK.ply
    test/res/ITKConeMeshConvertedToVTK.ply
    test/res/VTKPlaneMeshConvertedToITK.obj 
    test/res/VTKCubeMeshConvertedToITK.obj
    test/res/VTKArchMeshConvertedToITK.obj
    test/res/VTKSphereMeshConvertedToITK.obj
    test/res/VTKConeMeshConvertedToITK.obj
)
set(ORDERED_PCD_MESHER_TEST_RES
    test/res/PlaneOrderedPCDMesher.ply 
    test/res/CubeOrderedPCDMesher.ply
    test/res/ArchOrderedPCDMesher.ply
    test/res/SphereOrderedPCDMesher.ply
    test/res/ConeOrderedPCDMesher.ply
)
set(RAYTRACE_EXAMPLE_TEST_RES
    test/res/SavedPlaneRayTraceData.ply
    test/res/SavedCubeRayTraceData.ply
    test/res/SavedArchRayTraceData.ply
    test/res/SavedConeRayTraceData.ply
    test/res/SavedSphereRayTraceData.ply
)
set(RESAMPLED_POINT_CLOUD_TEST_RES
    test/res/PlaneResamplePointCloudExample.pcd
    test/res/CubeResamplePointCloudExample.pcd
    test/res/ArchResamplePointCloudExample.pcd
    test/res/SphereResamplePointCloudExample.pcd
    test/res/ConeResamplePointCloudExample.pcd
)
set(SMOOTH_NORMALS_TEST_RES
    test/res/PlaneWithSmoothedNormals.obj
    test/res/CubeWithSmoothedNormals.obj
    test/res/ArchWithSmoothedNormals.obj
    test/res/SphereWithSmoothedNormals.obj
    test/res/ConeWithSmoothedNormals.obj
)
set(SCALE_MESH_TEST_RES
    test/res/ScaledPlaneMesh.obj
    test/res/ScaledCubeMesh.obj
    test/res/ScaledArchMesh.obj
    test/res/ScaledSphereMesh.obj
    test/res/ScaledConeMesh.obj
)
set(ORDERED_RESAMPLING_TEST_RES
    test/res/OrderedResampling_Plane.obj
    test/res/OrderedResampling_Arch.obj
)
set(MESHING_TEST_RES
    ${ITK_VTK_TEST_RES}
    ${ORDERED_PCD_MESHER_TEST_RES}
    ${RAYTRACE_EXAMPLE_TEST_RES}
    ${RESAMPLED_POINT_CLOUD_TEST_RES}
    ${SMOOTH_NORMALS_TEST_RES}
    ${SCALE_MESH_TEST_RES}
    ${ORDERED_RESAMPLING_TEST_RES}
)
foreach(r ${MESHING_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()
endif(VC_BUILD_TESTS)
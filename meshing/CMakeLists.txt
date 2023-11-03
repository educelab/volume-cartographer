project(libvc_meshing VERSION ${VC_VERSION} LANGUAGES CXX)

set(srcs
    src/ACVD.cpp
    src/CalculateNormals.cpp
    src/DeepCopy.cpp
    src/ITK2VTK.cpp
    src/ScaleMesh.cpp
    src/SmoothNormals.cpp
    src/OrderedResampling.cpp
    src/OrderedPointSetMesher.cpp
    src/OrientNormals.cpp
    src/UVMapToITKMesh.cpp
    src/LaplacianSmooth.cpp
)
set(public_deps "")
set(private_deps "")
set(defs "")

add_library(vc_meshing ${srcs})
add_library(VC::meshing ALIAS vc_meshing)
target_compile_definitions(vc_meshing PUBLIC ${defs})
target_include_directories(vc_meshing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(vc_meshing
    PUBLIC
        VC::core
        ${public_deps}
    PRIVATE
        ITKTransform
        VTK::FiltersGeneral
        VTK::FiltersCore
        ACVD::DiscreteRemeshing
        ${private_deps}
)
target_compile_features(vc_meshing PUBLIC cxx_std_17)
set_target_properties(vc_meshing PROPERTIES
    VERSION "${PROJECT_VERSION}"
    EXPORT_NAME meshing
)

if(VC_INSTALL_LIBS)
install(
    TARGETS vc_meshing
    EXPORT ${targets_export_name}
    COMPONENT "Libraries"
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${include_install_dir}/meshing"
)
install(
    DIRECTORY "${include_install_dir}/meshing"
    DESTINATION "${include_install_dir}"
    COMPONENT "Libraries"
    FILES_MATCHING REGEX ".*\.(h|hpp)$"
)
endif()

### Testing ###
if(VC_BUILD_TESTS)

# Set source files
set(test_srcs
    test/CalculateNormalsTest.cpp
    test/OrderedResamplingTest.cpp
    test/ITK2VTKTest.cpp
    test/ScaleMeshTest.cpp
    test/SmoothNormalsTest.cpp
    test/OrderedPointSetMesherTest.cpp
    test/OrientNormalsTest.cpp
)

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname vc_meshing_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        VC::core
        VC::meshing
        VC::testing
        gtest_main
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
set(ORDERED_POINT_SET_MESHER_TEST_RES
    test/res/OrderedPointSetMesher_Plane.obj
    test/res/OrderedPointSetMesher_Arch.obj)

set(MESHING_TEST_RES
    ${ITK_VTK_TEST_RES}
    ${RAYTRACE_EXAMPLE_TEST_RES}
    ${RESAMPLED_POINT_CLOUD_TEST_RES}
    ${SMOOTH_NORMALS_TEST_RES}
    ${SCALE_MESH_TEST_RES}
    ${ORDERED_RESAMPLING_TEST_RES}
    ${ORDERED_POINT_SET_MESHER_TEST_RES}
)

foreach(r ${MESHING_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()
endif(VC_BUILD_TESTS)

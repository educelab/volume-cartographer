project(libvc_meshing VERSION ${VC_VERSION} LANGUAGES CXX)
set(module "meshing")
set(target "vc_${module}")

set(srcs
    src/CalculateNormals.cpp
    src/DeepCopy.cpp
    src/ITK2VTK.cpp
    src/RayTrace.cpp
    src/ScaleMesh.cpp
    src/SmoothNormals.cpp
    src/OrderedResampling.cpp
    src/OrderedPointSetMesher.cpp
)
set(deps
    VC::core
    Boost::filesystem
)
set(defs "")

# Conditional compilation of ACVD and Bullet stuff
if (VC_USE_ACVD OR VC_USE_OPTIONAL)
    list(APPEND srcs src/ACVD.cpp)
    list(APPEND deps acvd)
    list(APPEND defs VC_USE_ACVD)
endif()
if (VC_USE_BULLET OR VC_USE_OPTIONAL)
    list(APPEND srcs src/ITK2Bullet.cpp)
    list(APPEND deps bullet ${BULLET_LIBRARIES})
    list(APPEND defs VC_USE_BULLET)
endif()
if(VC_USE_VCG OR VC_USE_OPTIONAL)
    list(APPEND srcs src/QuadricEdgeCollapseDecimation.cpp)
    list(APPEND deps vcglib)
    list(APPEND defs VC_USE_VCGLIB)
endif()

add_library(${target} ${srcs})
add_library("${namespace}${module}" ALIAS "${target}")
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
)
set_target_properties(${target} PROPERTIES
    VERSION "${PROJECT_VERSION}"
    EXPORT_NAME "${module}"
)

if(VC_INSTALL_LIBS)
install(
    TARGETS ${target}
    EXPORT ${targets_export_name}
    COMPONENT "Libraries"
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${include_install_dir}/${module}"
)
install(
    DIRECTORY "${include_install_dir}/${module}"
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
    test/RayTraceTest.cpp
    test/ScaleMeshTest.cpp
    test/SmoothNormalsTest.cpp
    test/OrderedPointSetMesherTest.cpp
)

if(VC_USE_VCG OR VC_USE_OPTIONAL)
    list(APPEND test_srcs test/QuadricEdgeCollapseDecimationTest.cpp)
    set(QUADRIC_EDGE_COLLAPSE_DECIMATION_TEST_RES
            test/res/QuadricEdgeCollapse_Plane.obj
            test/res/QuadricEdgeCollapse_Arch.obj
            test/res/QuadricEdgeCollapse_Cone.obj
            test/res/QuadricEdgeCollapse_Cube.obj
            test/res/QuadricEdgeCollapse_Sphere.obj)
endif()

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        VC::core
        VC::meshing
        VC::testing
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
set(RAYTRACE_EXAMPLE_TEST_RES
    test/res/SavedPlaneRayTraceData.ply
    test/res/SavedCubeRayTraceData.ply
    test/res/SavedArchRayTraceData.ply
    test/res/SavedConeRayTraceData.ply
    test/res/SavedSphereRayTraceData.ply
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
    ${QUADRIC_EDGE_COLLAPSE_DECIMATION_TEST_RES}
    ${ORDERED_POINT_SET_MESHER_TEST_RES}
)

foreach(r ${MESHING_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()
endif(VC_BUILD_TESTS)

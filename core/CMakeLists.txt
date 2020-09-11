project(libvc_core VERSION ${VC_VERSION} LANGUAGES CXX)
set(module "core")
set(target "vc_${module}")

set(io_srcs
    src/OBJReader.cpp
    src/OBJWriter.cpp
    src/PLYReader.cpp
    src/PLYWriter.cpp
    src/SkyscanMetadataIO.cpp
    src/TIFFIO.cpp
)

set(math_srcs
    src/StructureTensor.cpp
)

set(neighborhood_srcs
    src/CuboidGenerator.cpp
    src/LineGenerator.cpp
)

set(shape_srcs
    src/ShapePrimitive.cpp
    src/Sphere.cpp
    src/Spiral.cpp
)

set(landmark_srcs
    src/VolumeLandmark.cpp
    src/PointLandmark.cpp
    src/PlaneLandmark.cpp
    src/PolylineLandmark.cpp
)

set(type_srcs
    src/DiskBasedObjectBaseClass.cpp
    src/HalfEdgeMesh.cpp
    src/Metadata.cpp
    src/PerPixelMap.cpp
    src/Render.cpp
    src/Reslice.cpp
    src/Segmentation.cpp
    src/Texture.cpp
    src/UVMap.cpp
    src/Volume.cpp
    src/VolumeMask.cpp
    src/VolumePkg.cpp
)

set(util_srcs
    src/MeshMath.cpp
    src/MemorySizeStringParser.cpp
    src/FormatStrToRegexStr.cpp
    src/BarycentricCoordinates.cpp
    src/ImageConversion.cpp
)

set(logging_srcs
    src/Logging.cpp
)

configure_file(src/Version.cpp.in Version.cpp)

add_library(${target}
    ${io_srcs}
    ${math_srcs}
    ${neighborhood_srcs}
    ${shape_srcs}
    ${landmark_srcs}
    ${type_srcs}
    ${util_srcs}
    ${logging_srcs}
    ${CMAKE_CURRENT_BINARY_DIR}/Version.cpp
)
add_library("${namespace}${module}" ALIAS "${target}")
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC
        Boost::filesystem
        Eigen3::Eigen
        opencv_core
        opencv_highgui
        opencv_imgproc
        opencv_imgcodecs
        ITKCommon
        ITKMesh
        ITKQuadEdgeMesh
        vtkCommonDataModel
        vtkCommonCore
        vtkIOLegacy
        VC::external
        spdlog::spdlog
    PRIVATE
        TIFF::TIFF
)

set_target_properties(${target} PROPERTIES
    VERSION "${PROJECT_VERSION}"
    EXPORT_NAME "${module}"
    POSITION_INDEPENDENT_CODE TRUE
)

## Python bindings ##
if(VC_BUILD_PYTHON_BINDINGS)
    set(python_srcs
        python/PyCore.cpp
        python/PyPerPixelMap.cpp
        python/PyReslice.cpp
        python/PyVolume.cpp
        python/PyVolumePkg.cpp
    )

    # This sets the module name in Python and must match the name in PyCore.cpp
    pybind11_add_module(Core ${python_srcs})
    target_link_libraries(Core
        PRIVATE
            VC::core
            VC::python
    )
    set_target_properties(Core PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/volcart"
    )

    set(PYBIND_TARGETS ${PYBIND_TARGETS} Core PARENT_SCOPE)
endif(VC_BUILD_PYTHON_BINDINGS)

## Install targets ##
if(VC_INSTALL_LIBS)
install(
    TARGETS "${target}"
    COMPONENT "Libraries"
    EXPORT "${targets_export_name}"
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    INCLUDES DESTINATION "${include_install_dir}/${module}"
    RUNTIME DESTINATION "bin"
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
set(test_srcs
    test/LRUCacheTest.cpp
    test/OBJWriterTest.cpp
    test/MetadataTest.cpp
    test/TextureTest.cpp
    test/UVMapTest.cpp
    test/PLYWriterTest.cpp
    test/PointSetTest.cpp
    test/PointSetIOTest.cpp
    test/OrderedPointSetTest.cpp
    test/OrderedPointSetIOTest.cpp
    test/PLYReaderTest.cpp
    test/FloatComparisonTest.cpp
    test/PerPixelMapTest.cpp
    test/OBJReaderTest.cpp
    test/NDArrayTest.cpp
    test/VolumeMaskTest.cpp
    test/LoggingTest.cpp
    test/SignalsTest.cpp
    test/IterationTest.cpp
)

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        VC::core
        VC::testing
        gtest_main
        gmock_main
    )
    add_test(
        NAME ${testname}
        WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
        COMMAND ${testname}
    )
endforeach()

# Set test resource files
set(COMMON_TEST_RES
    test/res/PlyWriter_Plane.ply
    test/res/GenericGradient.tif
    test/res/Texture.png
    test/res/vc_core_OBJReader_Invalid.obj
    test/res/vc_core_OBJReader_Textured.mtl
    test/res/vc_core_OBJReader_Textured.obj
    test/res/vc_core_OBJReader_TexturedWithNormals.mtl
    test/res/vc_core_OBJReader_TexturedWithNormals.obj
    test/res/vc_core_OBJReader_UntexturedWithNormals.obj
    test/res/vc_core_OBJReader_VariableVertInfo.obj
    test/res/vc_core_OBJReader_VariableVertInfo.mtl
    test/res/vc_core_OBJReader_PointCloud.obj
)

# Copy each test resource file
foreach(r ${COMMON_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()
endif()

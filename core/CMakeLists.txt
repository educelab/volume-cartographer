project(libvc_core VERSION 0.1 LANGUAGES CXX)
set(module "core")
set(target "vc_${module}")

set(io_srcs
    src/OBJReader.cpp
    src/OBJWriter.cpp
    src/PLYReader.cpp
    src/PLYWriter.cpp
)

set(shape_srcs
    src/ShapePrimitive.cpp
    src/Sphere.cpp
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
    src/VolumePkg.cpp
)

set(util_srcs
    src/MeshMath.cpp
)

set(srcs ${io_srcs} ${shape_srcs} ${type_srcs} ${util_srcs})

add_library(${target} ${srcs})
add_library("${namespace}${module}" ALIAS "${target}")
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC
        Boost::filesystem
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
        python/PyVolume.cpp
    )

    # This sets the module name in Python and must match the name in PyCore.cpp
    pybind11_add_module(Core ${python_srcs})
    target_link_libraries(Core
        PRIVATE
            VC::core
            VC::python
    )
endif(VC_BUILD_PYTHON_BINDINGS)

## Install targets ##
install(
    TARGETS "${target}"
    EXPORT "${targets_export_name}"
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    INCLUDES DESTINATION "${include_install_dir}/${module}"
    RUNTIME DESTINATION "bin"
)
install(
    DIRECTORY "${include_install_dir}/${module}"
    DESTINATION "${include_install_dir}"
    FILES_MATCHING REGEX ".*\.(h|hpp)$"
)

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
)

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        VC::core
        VC::testing
        Boost::unit_test_framework
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
    test/res/OnlyFaces.obj
    test/res/TexturedNoNormal.mtl
    test/res/TexturedNoNormal.obj
    test/res/TexturedPlane.mtl
    test/res/TexturedPlane.obj
    test/res/Texture.png
    test/res/UntexturedPlane.obj
    test/res/Vertices.obj
)

# Copy each test resource file
foreach(r ${COMMON_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()
endif()

project(libvc_core VERSION 0.1 LANGUAGES CXX)
set(target vc_core)

set(srcs
    src/HalfEdgeMesh.cpp
    src/Metadata.cpp
    src/PerPixelMap.cpp
    src/Rendering.cpp
    src/ShapePrimitive.cpp
    src/Slice.cpp
    src/Sphere.cpp
    src/Texture.cpp
    src/Volume.cpp
    src/getMemorySize.cpp
    src/meshMath.cpp
    src/objWriter.cpp
    src/PLYReader.cpp
    src/plyWriter.cpp
    src/VolumePkg.cpp
)

message("OpenCV3 libs: ${OpenCV_LIBS}")
add_library(${target} ${srcs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC
        Boost::filesystem
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        ITKCommon
        ITKMesh
        ITKQuadEdgeMesh
        vtkCommonDataModel
        vtkCommonCore
        vtkIOLegacy
        vc_external
)

install(TARGETS ${target}
    ARCHIVE DESTINATION lib COMPONENT Libraries
    LIBRARY DESTINATION lib COMPONENT Libraries
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX} COMPONENT Includes)
export(TARGETS ${target} FILE VCCommon.cmake)

### Testing ###
if(VC_BUILD_TESTS)

# Set source files
set(test_srcs
    test/LRUCacheTest.cpp
    test/ObjWriterTest.cpp
    test/MetadataTest.cpp
    test/TextureTest.cpp
    test/UVMapTest.cpp
    test/plyWriterTest.cpp
    test/PointSetTest.cpp
    test/PointSetIOTest.cpp
    test/OrderedPointSetTest.cpp
    test/OrderedPointSetIOTest.cpp
    test/PLYReaderTest.cpp
    test/FloatComparisonTest.cpp
    test/PerPixelMapTest.cpp
)

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        vc_core
        vc_testing
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
)

# Copy each test resource file
foreach(r ${COMMON_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()

endif(VC_BUILD_TESTS)

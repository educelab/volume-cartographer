project(libvc_common VERSION 0.1 LANGUAGES CXX)
set(target vc_common)

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
    src/ply2itk.cpp
    src/plyWriter.cpp
)

add_library(${target} ${srcs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # XXX This should go away - but PCL doesn't specify their targets in a
        # way that lets you do transitive dependencies.
        ${PCL_COMMON_INCLUDE_DIRS}
)
target_link_libraries(${target}
    PUBLIC
        Boost::filesystem
        opencv_core
        opencv_highgui
        opencv_imgproc
        ITKCommon
        ITKMesh
        ITKQuadEdgeMesh
        vtkCommonDataModel
        vtkCommonCore
        vtkIOLegacy
        # from deps/share/pcl-1.7/PCLConfig.cmake
        ${PCL_COMMON_LIBRARIES}
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
    test/PointTest.cpp
)

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        vc_common
        vc_testing
        Boost::unit_test_framework
    )
    target_compile_options(${testname}
        PUBLIC
            -Wno-c++11-narrowing
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

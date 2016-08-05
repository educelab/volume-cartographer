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
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})
export(TARGETS ${target} FILE VCCommon.cmake)

# Testing
set(test_srcs
    test/LRUCacheTest.cpp
    test/ObjWriterTest.cpp
    test/MetadataTest.cpp
    test/TextureTest.cpp
    test/UVMapTest.cpp
    test/plyWriterTest.cpp
)
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        vc_common
        vc_testing
        Boost::unit_test_framework
    )
    add_test(
        NAME ${testname}
        WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
        COMMAND ${testname}
    )
endforeach()

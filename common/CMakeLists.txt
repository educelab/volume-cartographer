project(libvc_common VERSION 0.1 LANGUAGES CXX)

set(VC_COMMON_SRCS
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

add_library(vc_common ${VC_COMMON_SRCS})
target_include_directories(vc_common
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # XXX This should go away - but PCL doesn't specify their targets in a
        # way that lets you do transitive dependencies.
        ${PCL_INCLUDE_DIRS}
)
message("VTK libs: ${VTK_LIBRARIES}")
target_link_libraries(vc_common
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
        pcl_common
)

install(TARGETS vc_common
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})
export(TARGETS vc_common FILE VCCommon.cmake)

# Testing
set(VC_COMMON_TEST_SRCS
    test/LRUCacheTest.cpp
    test/ObjWriterTest.cpp
    test/TextureTest.cpp
    test/UVMapTest.cpp
    test/plyWriterTest.cpp
)
foreach(src ${VC_COMMON_TEST_SRCS})
    get_filename_component(testname ${src} NAME_WE)
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        vc_common
        vc_testing
    )
    add_test(
        NAME ${testname}
        WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
        COMMAND ${testname}
    )
endforeach()

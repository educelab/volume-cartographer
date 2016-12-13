project(libvc_core VERSION 0.1 LANGUAGES CXX)
set(module "core")
set(target "vc_${module}")

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

add_library(${target} ${srcs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${include_install_dir}>
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
        vc_external
)

install(
    TARGETS "${target}"
    EXPORT "${targets_export_name}"
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    INCLUDES DESTINATION "${include_install_dir}/${module}"
    RUNTIME DESTINATION "bin"
)
install(
    DIRECTORY "include/${module}"
    DESTINATION "${include_install_dir}"
    FILES_MATCHING REGEX ".*\.(h|hpp)$"
)

### Testing ###
if(VC_BUILD_TESTS)
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
        test/PointTest.cpp
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
endif()

project(libvc_common VERSION 0.1 LANGUAGES CXX)

add_library(vc_common
	src/Texture.cpp
    src/Metadata.cpp
    src/ShapePrimitive.cpp
    src/Slice.cpp
    src/Sphere.cpp
    src/Volume.cpp
    src/getMemorySize.cpp
    src/objWriter.cpp
    src/ply2itk.cpp
    src/plyWriter.cpp
)

target_include_directories(vc_common
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # XXX This should go away - but PCL doesn't specify their targets in a
        # way that lets you do transitive dependencies.
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(vc_common
    PUBLIC
        Boost::filesystem
        OpenCV
        ITK
        VTK
        pcl_common
)

install(TARGETS vc_common
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})

export(TARGETS vc_common FILE VCCommon.cmake)

project(libvc_common VERSION 0.1 LANGUAGES CXX)

add_library(vc_common
    src/Metadata.cpp
	src/Texture.cpp
    src/Volume.cpp
    src/Slice.cpp
    src/objWriter.cpp
    src/plyWriter.cpp
    src/ply2itk.cpp
    src/ShapePrimitive.cpp
    src/Sphere.cpp
    src/getMemorySize.cpp)

# XXX This should go away - but PCL doesn't specify their targets in a way
# that lets you do transitive dependencies.
#include_directories(${PCL_INCLUDE_DIRS})

target_include_directories(vc_common
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PCL_INCLUDE_DIRS}
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/common>
        $<INSTALL_INTERFACE:include/common
)

target_link_libraries(vc_common
    PUBLIC
        Boost::filesystem
        OpenCV
        ITK
        VTK
        pcl_common
)

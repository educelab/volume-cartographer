project(libvc_meshing VERSION 0.1 LANGUAGES CXX)

set(VC_MESHING_SRCS
    src/deepCopy.cpp
    src/itk2vtk.cpp
    src/orderedPCDMesher.cpp
    src/rayTrace.cpp
    src/resamplePointCloud.cpp
    src/scaleMesh.cpp
    src/smoothNormals.cpp
    src/CalculateNormals.cpp
)
set(VC_MESHING_DEPENDENCIES
    vc_common
)
set(VC_MESHING_DEFINITIONS "")

# Conditional compilation of ACVD and Bullet stuff
if (VC_USE_ACVD)
    list(APPEND VC_MESHING_SRCS src/ACVD.cpp)
    list(APPEND VC_MESHING_DEPENDENCIES acvd)
    list(APPEND VC_MESHING_DEFINITIONS VC_USE_ACVD)
endif()
if (VC_USE_BULLET)
    list(APPEND VC_MESHING_SRCS src/itk2bullet.cpp src/cloth.cpp)
    list(APPEND VC_MESHING_DEPENDENCIES bullet)
    list(APPEND VC_MESHING_DEFINITIONS VC_USE_BULLET)
endif()

add_library(vc_meshing ${VC_MESHING_SRCS})

target_compile_definitions(vc_meshing
    PRIVATE
        ${VC_MESHING_DEFINITIONS}
)

target_include_directories(vc_meshing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(vc_meshing
    PUBLIC
        ${VC_MESHING_DEPENDENCIES} 
)

install(TARGETS vc_meshing
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})

export(TARGETS vc_meshing FILE VCMeshing.cmake)

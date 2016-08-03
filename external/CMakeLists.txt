project(libvc_external VERSION 0.1 LANGUAGES C CXX)

set(VC_EXTERNAL_SRCS
    src/eigenvalues.cc
    src/linear_solver.cc
    src/svd.cc
)

add_library(vc_external ${VC_EXTERNAL_SRCS})

target_include_directories(vc_external
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(vc_external
    PUBLIC
        eigen3
)

install(TARGETS vc_external
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})

export(TARGETS vc_external FILE VCExternal.cmake)

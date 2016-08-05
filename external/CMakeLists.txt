project(libvc_external VERSION 0.1 LANGUAGES C CXX)
set(target vc_external)

set(srcs
    src/eigenvalues.cc
    src/linear_solver.cc
    src/svd.cc
)

add_library(${target} ${srcs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC
        eigen3
)

install(TARGETS ${target}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})
export(TARGETS ${target} FILE VCExternal.cmake)

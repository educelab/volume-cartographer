project(libvc_texturing VERSION 0.1 LANGUAGES CXX)
set(target vc_texturing)

set(srcs
    src/compositeTexture.cpp
    src/compositeTextureV2.cpp
    src/simpleUV.cpp
    src/AngleBasedFlattening.cpp
)
set(public_deps
    vc_common
    vc_volumepkg
)
set(private_deps
    vc_meshing
    vc_external
)
set(defs "")

if (VC_USE_LIBIGL)
    list(APPEND srcs src/LeastSquaresConformalMapping.cpp)
    list(APPEND public_deps libigl)
    list(APPEND defs VC_USE_LIBIGL)
endif()
if (VC_USE_BULLET)
    list(APPEND srcs src/ClothModelingUVMapping.cpp)
    list(APPEND public_deps bullet ${BULLET_LIBRARIES})
    list(APPEND defs VC_USE_BULLET)
endif()

add_library(${target} ${srcs})
target_compile_definitions(${target} PUBLIC ${defs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC
        ${public_deps}
    PRIVATE
        ${private_deps}
)

install(TARGETS ${target}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})
export(TARGETS ${target} FILE VCTexturing.cmake)

set(test_srcs
    test/abfTest.cpp
    test/simpleUVTest.cpp
)
if (VC_USE_BULLET)
    list(APPEND test_srcs test/clothTest.cpp)
endif()
if (VC_USE_LIBIGL)
    list(APPEND test_srcs test/lscmTest.cpp)
endif()
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        vc_common
        vc_texturing
        vc_testing
        vc_meshing
        Boost::unit_test_framework
        ${public_deps}        
    )
    add_test(
        NAME ${testname}
        WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
        COMMAND ${testname}
    )
endforeach()
set(VC_TEXTURING_TEST_RES
    test/res/abf_Plane.obj
    test/res/abf_Plane_LSCMOnly.obj
    test/res/abf_Arch.obj
    test/res/abf_Arch_LSCMOnly.obj
)
if (VC_USE_BULLET)
    list(APPEND VC_TEXTURING_TEST_RES
        test/res/clothUV_Arch_Unfurl.obj
        test/res/clothUV_Arch_Collide.obj
        test/res/clothUV_Arch_Final.obj
    )
endif()
if (VC_USE_LIBIGL)
    list(APPEND VC_TEXTURING_TEST_RES
        test/res/lscm_Plane.obj
        test/res/lscm_Arch.obj
    )
endif()
foreach(r ${VC_TEXTURING_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()

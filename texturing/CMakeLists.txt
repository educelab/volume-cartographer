project(libvc_texturing VERSION ${VC_VERSION} LANGUAGES CXX)
set(module "texturing")
set(target "vc_${module}")

set(srcs
    src/CompositeTexture.cpp
    src/AngleBasedFlattening.cpp
    src/PPMGenerator.cpp
    src/IntersectionTexture.cpp
    src/IntegralTexture.cpp
    src/LayerTexture.cpp
    src/FlatteningAlgorithm.cpp
    src/ScaleMarkerGenerator.cpp
    src/OrthographicProjectionFlattening.cpp
    src/AlignmentMarkerGenerator.cpp
)
set(public_deps
    VC::core
)
set(private_deps
    opencv_imgproc
    VC::meshing
    VC::external
    vtkCommonCore
    vtkCommonDataModel
    vtkFiltersGeneral
)
set(defs "")

add_library(${target} ${srcs})
add_library("${namespace}${module}" ALIAS "${target}")
target_compile_definitions(${target} PUBLIC ${defs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target}
    PUBLIC ${public_deps}
    PRIVATE ${private_deps}
)
set_target_properties(${target} PROPERTIES
    VERSION "${PROJECT_VERSION}"
    EXPORT_NAME "${module}"
)

if(VC_INSTALL_LIBS)
install(
    TARGETS ${target}
    EXPORT ${targets_export_name}
    COMPONENT "Libraries"
    ARCHIVE DESTINATION "lib"
    LIBRARY DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${include_install_dir}/${module}"
)
install(
    DIRECTORY "${include_install_dir}/${module}"
    DESTINATION "${include_install_dir}"
    COMPONENT "Libraries"
    FILES_MATCHING REGEX ".*\.(h|hpp)$"
)
endif()

### Testing ###
if(VC_BUILD_TESTS)

# Set source files
set(test_srcs
    test/ABFTest.cpp
    test/PPMGeneratorTest.cpp
)

# Add a test executable for each src
foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        VC::texturing
        VC::testing
        gtest_main
        gmock_main
        ${public_deps}
    )
    add_test(
        NAME ${testname}
        WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
        COMMAND ${testname}
    )
endforeach()

# Set test resource files
set(VC_TEXTURING_TEST_RES
    test/res/abf_Plane.obj
    test/res/abf_Plane_LSCMOnly.obj
    test/res/abf_Arch.obj
    test/res/abf_Arch_LSCMOnly.obj
)

# Copy each test resource file
foreach(r ${VC_TEXTURING_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()

endif(VC_BUILD_TESTS)

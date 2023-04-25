project(libvc_graph VERSION ${VC_VERSION} LANGUAGES CXX)

set(srcs
    src/graph.cpp
    src/core.cpp
    src/meshing.cpp
    src/texturing.cpp
)

add_library(vc_graph ${srcs})
add_library(VC::graph ALIAS vc_graph)
target_include_directories(vc_graph
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(vc_graph
    PUBLIC
        VC::core
        VC::meshing
        VC::segmentation
        VC::texturing
        smgl::smgl
    PRIVATE
        nlohmann_json::nlohmann_json
)
target_compile_features(vc_graph PUBLIC cxx_std_17)

set_target_properties(vc_graph PROPERTIES
    VERSION ${PROJECT_VERSION}
    EXPORT_NAME graph
)

## Install targets ##
if(VC_INSTALL_LIBS)
    install(
        TARGETS vc_graph
        COMPONENT "Libraries"
        EXPORT "${targets_export_name}"
        ARCHIVE DESTINATION "lib"
        LIBRARY DESTINATION "lib"
        INCLUDES DESTINATION "${include_install_dir}/graph"
        RUNTIME DESTINATION "bin"
    )
    install(
        DIRECTORY "${include_install_dir}/graph"
        DESTINATION "${include_install_dir}"
        COMPONENT "Libraries"
        FILES_MATCHING REGEX ".*\.(h|hpp)$"
    )
endif()

### Testing ###
if(VC_BUILD_TESTS)
    set(test_srcs "")

    # Add a test executable for each src
    foreach(src ${test_srcs})
        get_filename_component(filename ${src} NAME_WE)
        set(testname vc_core_${filename})
        add_executable(${testname} ${src})
        target_link_libraries(${testname}
            VC::graph
            VC::testing
            gtest_main
            gmock_main
        )
        add_test(
            NAME ${testname}
            WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
            COMMAND ${testname}
        )
    endforeach()

    # Set test resource files
    set(COMMON_TEST_RES "")

    # Copy each test resource file
    foreach(r ${COMMON_TEST_RES})
        file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
    endforeach()
endif()

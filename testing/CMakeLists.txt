project(libvc_testing VERSION ${VC_VERSION} LANGUAGES CXX)

set(srcs
    src/ParsingHelpers.cpp
    src/TestingUtils.cpp
)
set(defs "")

add_library(vc_testing ${srcs})
add_library(VC::testing ALIAS vc_testing)
target_include_directories(vc_testing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(vc_testing
    PUBLIC
        VC::core
        ${VC_FS_LIB}
        opencv_core
    PRIVATE
        gtest
)
target_compile_definitions(vc_testing
    PUBLIC
        ${defs}
)
target_compile_features(vc_testing PUBLIC cxx_std_17)
set_target_properties(vc_testing PROPERTIES
    VERSION "${PROJECT_VERSION}"
    EXPORT_NAME testing
)

# Testing tests
# Add a test executable for each src
set(test_srcs
    test/ParsingHelpersTest.cpp
)

foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname vc_testing_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        VC::core
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

# Global files necessary for testing
set(VC_COMMON_TEST_RES
    test/res/Testing.volpkg
    test/res/OBJWriterPlaneData.obj
)
foreach(r ${VC_COMMON_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()

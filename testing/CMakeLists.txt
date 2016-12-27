project(libvc_testing VERSION 0.1 LANGUAGES CXX)
set(target vc_testing)

set(srcs
    src/ParsingHelpers.cpp
)
set(defs "")

if (NOT Boost_USE_STATIC_LIBS)
    list(APPEND defs BOOST_TEST_DYN_LINK)
endif()

add_library(${target} ${srcs})
target_include_directories(${target}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(${target} PUBLIC vc_core)
target_compile_definitions(${target} PUBLIC ${defs})

install(TARGETS ${target}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})
export(TARGETS ${target} FILE VCTesting.cmake)

# Testing tests
# Add a test executable for each src
set(test_srcs
    test/ParsingHelpersTest.cpp
)

foreach(src ${test_srcs})
    get_filename_component(filename ${src} NAME_WE)
    set(testname ${target}_${filename})
    add_executable(${testname} ${src})
    target_link_libraries(${testname}
        vc_core
        vc_testing
        Boost::unit_test_framework
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

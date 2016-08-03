project(libvc_testing VERSION 0.1 LANGUAGES CXX)

set(VC_TESTING_SRCS
    src/parsingHelpers.cpp
)
set(VC_TESTING_DEFINITIONS "")

if (NOT Boost_USE_STATIC_LIBS)
    list(APPEND VC_TESTING_DEFINITIONS BOOST_TEST_DYN_LINK)
endif()

add_library(vc_testing ${VC_TESTING_SRCS})
target_include_directories(vc_testing
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(vc_testing
    PUBLIC
        Boost::unit_test_framework
        vc_common
)
target_compile_definitions(vc_testing
    PUBLIC
        ${VC_TESTING_DEFINITIONS}
)

install(TARGETS vc_testing
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})
export(TARGETS vc_testing FILE VCTesting.cmake)

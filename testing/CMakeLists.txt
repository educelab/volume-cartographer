project(libvc_testing VERSION 0.1 LANGUAGES CXX)
set(target vc_testing)

set(srcs
    src/parsingHelpers.cpp
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
target_link_libraries(${target} PUBLIC vc_common)
target_compile_definitions(${target} PUBLIC ${defs})

install(TARGETS ${target}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION ${VC_INSTALL_INCLUDE_PREFIX})
export(TARGETS ${target} FILE VCTesting.cmake)

# Global files necessary for testing
set(VC_COMMON_TEST_RES
    res/Testing.volpkg
    res/OBJWriterPlaneData.obj
)
foreach(r ${VC_COMMON_TEST_RES})
    file(COPY ${r} DESTINATION ${EXECUTABLE_OUTPUT_PATH})
endforeach()

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(vc_test_OBJExporter examples/sampleTest.cpp)
target_link_libraries(vc_test_OBJExporter ${VC_LIBS} ${VC_REQ_LIBS} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
add_test(NAME OBJExporter WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin COMMAND vc_test_OBJExporter )
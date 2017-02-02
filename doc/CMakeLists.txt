# Doxygen dependency
find_package(Doxygen REQUIRED)

# Build Doxyfile
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    @ONLY
)

# Doxygen extra sources
set(VC_DOXY_FILES
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    ${CMAKE_CURRENT_SOURCE_DIR}/Modules.dox
)

# Add custom doc target (and add it to all target)
add_custom_target(
    doc ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${VC_DOXY_FILES}>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
)

# Install command
install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
    DESTINATION doc
    COMPONENT Documentation
)
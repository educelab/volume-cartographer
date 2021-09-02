# Set Doxygen options
set(DOXYGEN_DOT_IMAGE_FORMAT svg)
set(DOXYGEN_EXAMPLE_PATH ${PROJECT_SOURCE_DIR}/examples/src/)
set(DOXYGEN_EXTRACT_STATIC YES)
set(DOXYGEN_EXTRACT_PRIVATE YES)
set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_GENERATE_LATEX YES)
set(DOXYGEN_HTML_EXTRA_STYLESHEET ${PROJECT_SOURCE_DIR}/docs/style/doxygen_extra.css)
set(DOXYGEN_SOURCE_BROWSER YES)
set(DOXYGEN_TEMPLATE_RELATIONS YES)
set(DOXYGEN_QUIET YES)

set(DOXYGEN_CITE_BIB_FILES ${PROJECT_SOURCE_DIR}/docs/citations.bib)
set(DOXYGEN_IMAGE_PATH ${PROJECT_SOURCE_DIR}/docs/images/)

set(DOXYGEN_PROJECT_NAME "Volume Cartographer")
set(DOXYGEN_PROJECT_LOGO ${PROJECT_SOURCE_DIR}/docs/images/logo.svg)
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "${PROJECT_SOURCE_DIR}/README.md")

set(DOX_FILES
    ${PROJECT_SOURCE_DIR}/CONTRIBUTING.md
    ${PROJECT_SOURCE_DIR}/README.md
    ${PROJECT_SOURCE_DIR}/docs/Modules.dox
)

set(DOX_DIRS
    ${PROJECT_SOURCE_DIR}/docs/pages/
    ${PROJECT_SOURCE_DIR}/core/include/
    ${PROJECT_SOURCE_DIR}/meshing/include/
    ${PROJECT_SOURCE_DIR}/segmentation/include/
    ${PROJECT_SOURCE_DIR}/texturing/include/
    ${PROJECT_SOURCE_DIR}/graph/include/
)

set(DOXYGEN_EXCLUDE
    ${PROJECT_SOURCE_DIR}/core/include/vc/core/third_party
)

set(DOXYGEN_STRIP_FROM_INC_PATH ${DOX_DIRS})
set(DOXYGEN_STRIP_FROM_PATH ${DOX_DIRS})

if(VC_BUILD_APPS OR VC_BUILD_UTILS)
    list(APPEND DOX_DIRS ${PROJECT_SOURCE_DIR}/app_support/include/)
    list(APPEND DOXYGEN_STRIP_FROM_INC_PATH ${PROJECT_SOURCE_DIR}/app_support/include/)
    list(APPEND DOXYGEN_STRIP_FROM_PATH ${PROJECT_SOURCE_DIR}/app_support/include/)
endif()

if(VC_BUILD_EXPERIMENTAL)
    list(APPEND DOX_DIRS ${PROJECT_SOURCE_DIR}/experimental/include/)
    list(APPEND DOXYGEN_STRIP_FROM_INC_PATH ${PROJECT_SOURCE_DIR}/experimental/include/)
    list(APPEND DOXYGEN_STRIP_FROM_PATH ${PROJECT_SOURCE_DIR}/experimental/include/)
endif()

if(VC_BUILD_PYTHON_BINDINGS)
    list(APPEND DOX_DIRS ${PROJECT_SOURCE_DIR}/python/include/)
    list(APPEND DOXYGEN_STRIP_FROM_INC_PATH ${PROJECT_SOURCE_DIR}/python/include/)
    list(APPEND DOXYGEN_STRIP_FROM_PATH ${PROJECT_SOURCE_DIR}/python/include/)
endif()

doxygen_add_docs(
    docs
    ${DOX_FILES}
    ${DOX_DIRS}
    COMMENT "Generate Doxygen documentation"
)

# Install command
if(VC_INSTALL_DOCS)
install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
    DESTINATION doc
    COMPONENT Documentation
)
endif()

FetchContent_Declare(
    smgl
    GIT_REPOSITORY https://gitlab.com/educelab/smgl.git
    GIT_TAG 437979c3
    CMAKE_CACHE_ARGS
        -DSMGL_BUILD_JSON:BOOL=ON
        -DSMGL_USE_BOOSTFS:BOOL=ON
        -DSMGL_BUILD_TESTS:BOOL=OFF
        -DSMGL_BUILD_DOCS:BOOL=OFF
)

FetchContent_GetProperties(smgl)
if(NOT smgl_POPULATED)
    set(SMGL_BUILD_JSON ON CACHE INTERNAL "")
    set(SMGL_USE_BOOSTFS ${VC_USE_BOOSTFS} CACHE INTERNAL "")
    set(SMGL_BUILD_TESTS OFF CACHE INTERNAL "")
    set(SMGL_BUILD_DOCS OFF CACHE INTERNAL "")
    FetchContent_Populate(smgl)
    add_subdirectory(${smgl_SOURCE_DIR} ${smgl_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

FetchContent_Declare(
    smeagol
    GIT_REPOSITORY https://gitlab.com/educelab/smeagol.git
    GIT_TAG v0.9
    CMAKE_CACHE_ARGS
        -DSMGL_BUILD_JSON:BOOL=ON
        -DSMGL_USE_BOOSTFS:BOOL=ON
        -DSMGL_BUILD_TESTS:BOOL=OFF
        -DSMGL_BUILD_DOCS:BOOL=OFF
)

FetchContent_GetProperties(smeagol)
if(NOT smeagol_POPULATED)
    set(SMGL_BUILD_JSON ON CACHE INTERNAL "")
    set(SMGL_USE_BOOSTFS ${VC_USE_BOOSTFS} CACHE INTERNAL "")
    set(SMGL_BUILD_TESTS OFF CACHE INTERNAL "")
    set(SMGL_BUILD_DOCS OFF CACHE INTERNAL "")
    FetchContent_Populate(smeagol)
    add_subdirectory(${smeagol_SOURCE_DIR} ${smeagol_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

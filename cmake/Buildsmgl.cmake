FetchContent_Declare(
    smgl
    GIT_REPOSITORY https://github.com/educelab/smgl.git
    GIT_TAG v0.11.0-rc.2
    EXCLUDE_FROM_ALL
)
set(SMGL_BUILD_JSON ${VC_BUILD_JSON} CACHE INTERNAL "")
set(SMGL_USE_BOOSTFS ${VC_USE_BOOSTFS} CACHE INTERNAL "")
set(SMGL_BUILD_TESTS OFF CACHE INTERNAL "")
set(SMGL_BUILD_DOCS OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(smgl)

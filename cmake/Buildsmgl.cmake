FetchContent_Declare(
    smgl
    GIT_REPOSITORY https://gitlab.com/educelab/smgl.git
    GIT_TAG v0.10.1
    EXCLUDE_FROM_ALL
)
set(SMGL_BUILD_JSON ON CACHE INTERNAL "")
set(SMGL_USE_BOOSTFS ${VC_USE_BOOSTFS} CACHE INTERNAL "")
set(SMGL_BUILD_TESTS OFF CACHE INTERNAL "")
set(SMGL_BUILD_DOCS OFF CACHE INTERNAL "")
FetchContent_MakeAvailable(smgl)

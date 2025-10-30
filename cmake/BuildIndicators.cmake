FetchContent_Declare(
    indicators
    GIT_REPOSITORY https://github.com/p-ranav/indicators.git
    GIT_TAG 222382c
    PATCH_COMMAND patch -p1 -i ${CMAKE_SOURCE_DIR}/cmake/patches/Indicators-CMakeMinVer.patch
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(indicators)

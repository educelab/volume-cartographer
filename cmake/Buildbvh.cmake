FetchContent_Declare(
  bvh
  GIT_REPOSITORY https://github.com/madmann91/bvh.git
  GIT_TAG 2fd0db6
  PATCH_COMMAND patch -p1 -i ${CMAKE_SOURCE_DIR}/cmake/patches/bvh-CMakeMinVer.patch
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(bvh)
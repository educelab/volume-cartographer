# Declare the project
FetchContent_Declare(
  bvh
  GIT_REPOSITORY https://github.com/madmann91/bvh.git
  GIT_TAG 19dde37
)

# Populate the project but exclude from all
FetchContent_GetProperties(bvh)
if(NOT bvh_POPULATED)
  FetchContent_Populate(bvh)
  add_subdirectory(${bvh_SOURCE_DIR} ${bvh_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
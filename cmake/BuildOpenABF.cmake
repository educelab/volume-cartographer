option(VC_BUILD_OPENABF "Build in-source OpenABF" on)
if(VC_BUILD_OPENABF)
  # Declare the project
  FetchContent_Declare(
      openabf
      GIT_REPOSITORY https://gitlab.com/educelab/OpenABF.git
      GIT_TAG 34b717a5
  )

  # Populate the project but exclude from all
  FetchContent_GetProperties(openabf)
  if(NOT openabf_POPULATED)
      FetchContent_Populate(openabf)
      add_subdirectory(${openabf_SOURCE_DIR} ${openabf_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
else()
    find_package(OpenABF 1.0 REQUIRED)
endif()

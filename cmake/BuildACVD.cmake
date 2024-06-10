option(VC_BUILD_ACVD "Build in-source ACVD" ON)
if(VC_BUILD_ACVD)
  # Declare the project
  FetchContent_Declare(
      acvd
      GIT_REPOSITORY https://gitlab.com/educelab/acvd.git
      GIT_TAG 9efaeb4d
  )

  # Populate the project but exclude from all
  FetchContent_GetProperties(acvd)
  if(NOT acvd_POPULATED)
    FetchContent_Populate(acvd)
    add_subdirectory(${acvd_SOURCE_DIR} ${acvd_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
else()
  find_package(ACVD 1.2 REQUIRED)
endif()

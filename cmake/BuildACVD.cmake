option(VC_BUILD_ACVD "Build in-source ACVD" ON)
if(VC_BUILD_ACVD)
  # Declare the project
  FetchContent_Declare(
      acvd
      GIT_REPOSITORY https://gitlab.com/educelab/acvd.git
      GIT_TAG v1.2.1
      EXCLUDE_FROM_ALL
  )
  FetchContent_MakeAvailable(acvd)
else()
  find_package(ACVD 1.2 REQUIRED)
endif()

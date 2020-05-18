option(VC_USE_ACVD "Use ACVD library" on)
cmake_dependent_option(VC_BUILD_ACVD "Build in-source ACVD" off "VC_USE_ACVD OR VC_USE_OPTIONAL" off)
if (VC_USE_ACVD OR VC_USE_OPTIONAL)
  if(VC_BUILD_ACVD)
    # Declare the project
    FetchContent_Declare(
            acvd
            GIT_REPOSITORY https://github.com/csparker247/ACVD.git
            GIT_TAG v1.1.2
    )
    
    # Populate the project but exclude from all
    FetchContent_GetProperties(acvd)
    if(NOT acvd_POPULATED)
      FetchContent_Populate(acvd)
      add_subdirectory(${acvd_SOURCE_DIR} ${acvd_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
  else()
    find_package(ACVD 1.1.2 REQUIRED)
  endif()
endif()

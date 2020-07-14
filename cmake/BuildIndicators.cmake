# Declare the project
FetchContent_Declare(
    indicators
    GIT_REPOSITORY https://github.com/p-ranav/indicators.git
    GIT_TAG v1.9
)

# Populate the project but exclude from all
FetchContent_GetProperties(indicators)
if(NOT indicators_POPULATED)
    FetchContent_Populate(indicators)
    add_subdirectory(${indicators_SOURCE_DIR} ${indicators_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

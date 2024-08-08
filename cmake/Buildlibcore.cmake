FetchContent_Declare(
    libcore
    GIT_REPOSITORY https://github.com/educelab/libcore.git
    GIT_TAG split-multichar-delim
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(libcore)
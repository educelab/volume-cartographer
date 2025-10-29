FetchContent_Declare(
    openabf
    GIT_REPOSITORY https://gitlab.com/educelab/OpenABF.git
    GIT_TAG support-eigen5
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(openabf)

# Declare the project
FetchContent_Declare(
    threadpool
    GIT_REPOSITORY https://github.com/bshoshany/thread-pool.git
    GIT_TAG v3.5.0
)
FetchContent_MakeAvailable(threadpool)

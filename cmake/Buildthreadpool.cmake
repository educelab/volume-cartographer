FetchContent_Declare(
    threadpool
    GIT_REPOSITORY https://github.com/bshoshany/thread-pool.git
    GIT_TAG v4.1.0
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(threadpool)
# Also enable warnings project-wide
set(project_warnings
    -Wall
    -Wextra
    -Wno-missing-braces
    -Wno-c++11-narrowing
    -Wimplicit-fallthrough
)
add_compile_options(${project_warnings})

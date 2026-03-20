# C++ Style Guide — Volume Cartographer

## Enforcement

**clang-format is the authoritative formatter.** Before submitting any PR, run:

```shell
git clang-format develop
```

CI will reject PRs that fail clang-format compliance. The `.clang-format` file in the repo root defines all formatting rules — do not override it inline.

## Naming Conventions

| Element | Convention | Example |
|---------|------------|---------|
| Namespace | `snake_case` | `volcart::texturing` |
| Class/Struct | `PascalCase` | `VolumePkg`, `PerPixelMap` |
| Function/Method | `camelCase` | `getVolume()`, `setSliceData()` |
| Variable | `camelCase` | `sliceIndex`, `uvMap` |
| Member variable | `camelCase` with no prefix | `width_`, `data_` (trailing underscore for private) |
| Constant / enum value | `UPPER_SNAKE_CASE` or `PascalCase` | `MAX_CACHE_SIZE` |
| File | `PascalCase.hpp` / `PascalCase.cpp` | `VolumePkg.hpp` |

## Namespaces

All library code lives under `volcart::`. Sub-module namespaces follow the module name:

```cpp
namespace volcart::segmentation { ... }
namespace volcart::texturing { ... }
```

Do not use `using namespace volcart;` or `using namespace std;` in headers.

## Headers

- Use `#pragma once` (not include guards)
- Keep headers minimal — forward-declare where possible
- Public API headers go in `{module}/include/VC/{Module}/`
- Internal headers go in `{module}/src/`

## Classes

- Prefer composition over inheritance except for algorithm abstractions (`FlatteningAlgorithm`, `TexturingAlgorithm`, etc.)
- Abstract base classes use pure virtual interfaces; prefer non-virtual destructors unless polymorphic deletion is needed
- Use `= default` / `= delete` explicitly for special member functions

## Memory and Ownership

- Prefer smart pointers (`std::unique_ptr`, `std::shared_ptr`) over raw owning pointers
- Pass by `const&` for read-only access, by value for sinks, by `&` for out-params only when necessary
- Large data (volumes, meshes) is always accessed via handles/references, never copied by value

## Error Handling

- Use exceptions for unrecoverable errors; return `std::optional` or error codes for expected failure paths
- Do not silently swallow exceptions

## Modern C++

- Target C++17
- Prefer range-based for loops, structured bindings, `if constexpr`, `std::filesystem`
- Use `[[nodiscard]]` on functions whose return values must not be ignored
- Avoid raw arrays; use `std::array` or `std::vector`

## Algorithm Implementation

- New segmentation algorithms inherit from the abstract base in `segmentation/`
- New flattening algorithms inherit from `FlatteningAlgorithm` in `texturing/`
- New texturing algorithms inherit from `TexturingAlgorithm`
- Register algorithms in the appropriate factory if one exists

## Testing

- Every new class or algorithm gets a corresponding `{Module}_{FeatureName}Test.cpp`
- Tests use GoogleTest (`TEST`, `TEST_F`, `EXPECT_*`, `ASSERT_*`)
- Test binaries are named `vc_{module}_{TestName}` and registered with CTest
- Do not use `main()` in test files — use `gtest_main`

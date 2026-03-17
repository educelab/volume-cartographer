# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```shell
# Configure (Ninja recommended for development)
cmake -S . -B build/ -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Build
cmake --build build/

# Configure with tests enabled
cmake -S . -B build/ -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DVC_BUILD_TESTS=ON

# Run all tests
ctest -V --test-dir build/

# Run a single test by name
ctest -V --test-dir build/ -R vc_core_UVMapTest

# Run a test binary directly
./build/bin/vc_core_LRUCacheTest
```

Key CMake options: `VC_BUILD_APPS` (ON), `VC_BUILD_GUI` (ON), `VC_BUILD_TESTS` (OFF), `VC_BUILD_PYTHON_BINDINGS` (OFF), `VC_BUILD_EXAMPLES` (OFF), `VC_PREBUILT_LIBS` (OFF), `BUILD_SHARED_LIBS`.

## Code Style

Run `git clang-format develop` before submitting PRs. CI enforces clang-format compliance.

## Architecture

The project is a layered C++ library + application suite under the `volcart` namespace, processing CT scan volumes to produce flat texture images of ancient scrolls/documents.

**Dependency stack (bottom to top):**

1. **`VC::core`** — All fundamental types: `VolumePkg` (`.volpkg` directory container), `Volume` (LRU-cached TIFF slice stack), `Segmentation` (surface point cloud), `PerPixelMap`, `UVMap`, mesh types, IO readers/writers (OBJ/PLY/TIFF), math utilities, logging (spdlog), signals.

2. **`VC::meshing`** / **`VC::segmentation`** / **`VC::texturing`** — Algorithm libraries that depend only on `VC::core`.
   - `meshing`: ITK↔VTK conversion, ACVD remeshing, normal calculation
   - `segmentation`: LRPS (LocalResliceParticleSim), STPS, OpticalFlow, FloodFill — all inherit from abstract base classes
   - `texturing`: ABF/LSCM/Orthographic flattening (via OpenABF), PPM generation, volume sampling — algorithms inherit from `FlatteningAlgorithm` / `TexturingAlgorithm`

3. **`VC::graph`** — Node-graph pipeline abstraction over the above, using the `smgl` library.

4. **`apps/`** / **`utils/`** — CLI tools and Qt6 GUI (`VC.app`) linking against the libraries.

**Typical data pipeline:** `vc_packager` → segment with `vc_segment`/GUI → `vc_mesher` → `vc_render`/`vc_generate_ppm` → flat texture image.

**Test model:** Each `*Test.cpp` compiles to its own binary named `{module}_{TestName}` (e.g., `vc_segmentation_FittedCurveTest`), registered individually with CTest. Tests run with working directory `build/bin/`.

**In-source dependencies** fetched via CMake `FetchContent`: OpenABF, bvh, smgl, libcore, ACVD, indicators, googletest. Custom `Build*.cmake` wrappers live in `cmake/`.

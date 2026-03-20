# Tech Stack — Volume Cartographer

## Primary Languages

- **C++17** — Core library, all algorithm modules, CLI tools, Qt6 GUI
- **Python 3** — Optional bindings via pybind11 (`VC_BUILD_PYTHON_BINDINGS=ON`)

## Build System

- **CMake** (minimum version per `CMakeLists.txt`) with **Ninja** recommended for development
- In-source dependencies fetched via `FetchContent` (see below)

## GUI

- **Qt6** — Desktop application (`VC.app`, `apps/VC/`)

## Core Dependencies

| Library | Role |
|---------|------|
| ITK | Image processing, mesh types |
| VTK | Mesh processing, rendering pipeline |
| OpenCV | Image processing utilities |
| Boost | General utilities |
| Eigen | Linear algebra |
| spdlog | Logging |
| pybind11 | Python bindings |

## In-Source Dependencies (FetchContent)

| Library | Role |
|---------|------|
| OpenABF | ABF++/LSCM UV flattening |
| smgl | Node-graph pipeline abstraction |
| bvh | Bounding volume hierarchy |
| libcore | Supporting core utilities |
| ACVD | Anisotropic mesh remeshing |
| indicators | CLI progress bars |
| googletest | Unit testing framework |

## Storage Format

- **`.volpkg`** — Custom directory-based container: JSON metadata (`config.json`) + TIFF slice stacks for volumes, point cloud files for segmentations, PPM files for per-pixel maps.

## Distribution

- **Built from source** via CMake (primary)
- **Homebrew** tap + **GitHub Releases** for macOS prebuilt packages
- **Docker-based CI** (GitHub Actions) producing prebuilt binaries for releases

## CI/CD

- GitHub Actions for build, test, and release workflows
- clang-format enforcement on all PRs (`git clang-format develop`)

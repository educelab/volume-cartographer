[![Volume Cartographer](docs/images/banner.svg)](https://github.com/educelab/volume-cartographer)

**Volume Cartographer** is a toolkit and set of cross-platform C++ libraries for
virtually unwrapping volumetric datasets. It was designed to recover text from
CT scans of ancient, badly damaged manuscripts, but can be applied in many
volumetric analysis applications.

## Getting started
New to Volume Cartographer? A great place to get started with virtual unwrapping
is the [tutorial](https://scrollprize.org/tutorial3) put together by
the [Vesuvius Challenge](https://scrollprize.org/).

You can also browse our [application list](docs/pages/apps-list.md) for an
overview of our available applications and utilities.

## Installation
### Using Homebrew
We provide pre-built binaries for our tools through our
[Homebrew Casks tap](https://github.com/educelab/homebrew-casks):

```shell
brew install --no-quarantine educelab/casks/volume-cartographer
```

Our binaries are signed with a generic signature and thus do not pass macOS
Gatekeeper on Apple Silicon devices without explicit approval. Since many of
our tools are run from the command line, we suggest installing with Homebrew's
`--no-quarantine` flag.

The main `VC.app` GUI will be installed to `/Applications/` and the command line
tools should be immediately available in Terminal:

```shell
vc_render --help
```

### Using Docker
We provide multi-architecture Docker images in the GitHub Container Registry.
Simply pull our container and Docker will select the appropriate image for your
host platform:

```shell
# Pull the latest release
docker pull ghcr.io/educelab/volume-cartographer:latest

# Pull the latest edge version
docker pull ghcr.io/educelab/volume-cartographer:edge

# Pull a specific version
docker pull ghcr.io/educelab/volume-cartographer:2.24.0
```

Tools can be launched directly using `docker run`:

```shell
$ docker run ghcr.io/educelab/volume-cartographer vc_render --help
Usage:

General Options:
  -h [ --help ]                         Show this message
  --cache-memory-limit arg              Maximum size of the slice cache in
                                        bytes. Accepts the suffixes:
                                        (K|M|G|T)(B). Default: 50% of the total
                                        system memory.
  --log-level arg (=info)               Options: off, critical, error, warn,
                                        info, debug
...
```

To run the GUI tools, you must additionally set up
[X11 forwarding from the container](docs/pages/running-gui-tools.md).

### From source
#### Supported platforms
This project is primarily developed and tested on macOS and Debian/Ubuntu
systems. Though it should compile with any C++17 compiler using the Itanium ABI,
this has not been tested on Windows. We are accepting contributions to
explicitly support other platforms.

#### Dependencies
**Required**
* C++17 compiler which uses the Itanium ABI
* CMake 3.24+
* OpenCV 3+
* Insight Toolkit (itk) 4.10+
* Visualization Toolkit (vtk) 7 or 8
* [ACVD](https://gitlab.com/educelab/acvd) mesh simplification library
* [libtiff](https://gitlab.com/libtiff/libtiff) 4.0+
* Eigen3 3.2+
* spdlog 1.4.2+
* Boost Program Options 1.58+: Required if building applications or utilities.
* Qt 6.3+: Required if building GUI applications or utilities.

**Optional**
* Boost Filesystem 1.58+
    - This project will automatically check if the compiler provides
    `std::filesystem`. If it is not found, then Boost Filesystem is required.
    This behavior can be controlled with the `VC_USE_BOOSTFS` CMake flag.
* [Doxygen](https://www.doxygen.nl/): Required to build
documentation.
* [pybind11](https://github.com/pybind/pybind11): Required to build Python
bindings.

##### (macOS) Homebrew-provided dependencies
In principle, Homebrew can be used to install all of Volume Cartographer's
dependencies. However at the time of this writing, the `vtk` brew package links
against Qt5 while the Volume Cartographer GUIs link against Qt6. This will lead
to linking errors when compiling this project. To use the Homebrew version of
VTK, you must disable compilation of the VC GUI apps with the
`-DVC_BUILD_GUI=OFF` CMake flag. Otherwise, you must build VTK from source or
follow the instructions for
[building vc-deps dependencies](#optional-use-vc-deps-dependencies).

#### Compilation
This project is built and installed using the CMake build system. If you have
already installed the dependencies listed above, compilation should be as simple
as:

```shell
git clone https://github.com/educelab/volume-cartographer.git
cd volume-cartographer
cmake -S . -B build/ -DCMAKE_BUILD_TYPE=Release
cmake --build build/
```

Many `volume-cartographer` libraries can be built in parallel, and compilation
times will be improved by running `cmake --build build/ -j4`. Alternatively,
you can use CMake to generate [Ninja](https://ninja-build.org/) build system
files:
```shell
cmake -S . -B build/ -GNinja -DCMAKE_BUILD_TYPE=Release
cmake --build build/  # automatically builds in parallel with Ninja
```

To install the compiled software and libraries to the `CMAKE_INSTALL_PREFIX`,
run the `install` target:
```shell
cmake --install build/ # --prefix ~/custom/install/prefix/
```

##### (Optional) Use vc-deps dependencies
To assist with installing dependencies, we have created the
[vc-deps project](https://github.com/educelab/vc-deps). While this project can
be used on its own to install volume-cartographer dependencies to the system, it
is also available as a git submodule within `volume-cartographer`. Note that
`vc-deps` **does not** install CMake or Qt.

To build and link against the in-source `vc-deps` libraries, run the following:
```shell
# Get the source code plus all submodules
git clone --recursive https://github.com/educelab/volume-cartographer.git
cd volume-cartographer

# If you already cloned volume-cartographer
# git submodule update --init

# (macOS only)
# brew install boost qt
# brew unlink qt

# Build vc-deps
cmake -S vc-deps/ -B vc-deps/build/ -DCMAKE_BUILD_TYPE=Release  # -DVCDEPS_BUILD_BOOST=OFF (if Boost already installed)
cmake --build vc-deps/build/

# Build volume-cartographer
cmake -S . -B build/ -DCMAKE_BUILD_TYPE=Release -DVC_PREBUILT_LIBS=ON
cmake --build build/
```

##### Linking against Qt
It might be necessary to point CMake to your Qt installation. For example:
```shell
# macOS (Apple Silicon), Qt6 installed via Homebrew
cmake -S . -B build/ -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt/lib/cmake/

# macOS (Intel), Qt6 installed via Homebrew
cmake -S . -B build/ -DCMAKE_PREFIX_PATH=/usr/local/opt/qt/lib/cmake/

# Ubuntu, Qt6 installed from source
cmake -S . -B build/ -DCMAKE_PREFIX_PATH=/usr/local/Qt-6.4.2/lib/cmake/
```

#### Unit tests
This project is instrumented with unit tests using the Google Test framework.
To enable test compilation, set the `VC_BUILD_TESTS` flag to on:
```shell
cmake -S . -B build/ -DVC_BUILD_TESTS=ON
```

Tests can then be run using CTest or by running the `test` target:
```shell
# Print summary output with the test target
cmake --build build/ --target test

# Print verbose output with ctest
ctest -V --test-dir build/
```

## API Documentation
Visit our API documentation
[here](https://educelab.gitlab.io/volume-cartographer/docs/).

Library documentation is built using Doxygen and can be enabled/disabled by
setting the `VC_BUILD_DOCS` flag. This requires Doxygen and optionally Graphviz.
This option is unavailable if Doxygen is not found. Documentation will be
installed with the `install` target if the `VC_INSTALL_DOCS` flag is enabled.

```shell
cmake -S . -B build/ -DVC_BUILD_DOCS=ON -DVC_INSTALL_DOCS=ON
```

## Python bindings
We currently maintain limited Python binding support through pybind11. They are
a work-in-progress and should not be used in production code.

Bindings can be built and installed by setting the
`VC_BUILD_PYTHON_BINDINGS` and `VC_INSTALL_PYTHON_BINDINGS` flags:
```shell
cmake -S . -B build/ -DVC_BUILD_PYTHON_BINDINGS=ON -DVC_INSTALL_PYTHON_BINDINGS=ON
```

To use these bindings in Python after installation, import from the
`volcart` package:
```python
import volcart.Core as c
import numpy as np

vpkg = c.VolumePkg('/path/to/package.volpkg')
vol = vpkg.volume()
r = vol.reslice(np.array([0,0,0]))
```

__NOTE:__ Python modules are built as shared libraries, regardless of the
`BUILD_SHARED_LIBS` flag set by this project. This can cause problems if the
Volume Cartographer dependencies are not built as shared libraries. Either
install the shared versions of these libraries (preferred) or compile static
libraries with position independent code (PIC).

If using `vc-deps` to build the dependency libraries, set the appropriate CMake
flags:
```shell
# Build shared libraries (preferred)
cmake -S . -B vc-deps/build/ -DBUILD_SHARED_LIBS=ON

# Build static libraries with PIC
cmake -S . -B vc-deps/build/ -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
```

## Contributing

See [CONTRIBUTING](CONTRIBUTING.md).

## License
Except where otherwise indicated, the software in this repository is licensed
under the [GNU General Public License v3.0](LICENSE). This project is free
software: you can redistribute it and/or modify it under the terms of the GPLv3
or (at your option) any later version.

This project incorporates software from many excellent external libraries and
projects. Please see [NOTICE](NOTICE) for more information about the licensing
terms of these projects.

**Volume Cartographer** and the project logo and banner graphics are trademarks
of EduceLab.

## Citation and references
If you use Volume Cartographer in your research, please cite this repository
in your publication using
[our Zenodo record](https://doi.org/10.5281/zenodo.4604881). For more
information about the concepts of virtual unwrapping, please see the following
publications:
* William Brent Seales et al. “From damage to discovery via virtual unwrapping:
  Reading the scroll from En-Gedi”. In: _Science Advances_ 2.9 (2016).
  doi: 10.1126/sciadv.1601247. [[link]](https://www.science.org/doi/10.1126/sciadv.1601247)
* Clifford Seth Parker, William Brent Seales, and Pnina Shor. “Quantitative
  Distortion Analysis of Flattening Applied to the Scroll from En-Gedi”. In:
  Art & Archaeology, 2nd International Conference. 2016.
  [[link]](https://arxiv.org/abs/2007.15551)
* W Brent Seales and Daniel Delattre. “Virtual unrolling of carbonized
  Herculaneum scrolls: Research Status (2007–2012)”. In: Cronache Ercolanesi 43
  (2013), pp. 191–208.
* Ryan Baumann, Dorothy Carr Porter, and W Brent Seales. “The use of micro-ct
  in the study of archaeological artifacts”. In: 9th International Conference
  on NDT of Art. 2008, pp. 1–9. [[link]](https://www.ndt.net/article/art2008/papers/244Seales.pdf)
* Yun Lin and W Brent Seales. “Opaque document imaging: Building images of
  inaccessible texts”. In: Computer Vision, 2005. ICCV 2005. Tenth IEEE
  International Conference on. Vol. 1. IEEE. 2005, pp. 662–669.
* W Brent Seales and Yun Lin. “Digital restoration using volumetric scanning”.
  In: Digital Libraries, 2004. Proceedings of the 2004 Joint ACM/IEEE
  Conference on. IEEE. 2004, pp. 117–124.

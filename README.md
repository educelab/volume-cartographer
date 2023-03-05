[![Volume Cartographer](docs/images/banner.svg)](https://github.com/educelab/volume-cartographer)

**Volume Cartographer** is a cross-platform C++ library and toolkit for
virtually unwrapping volumetric datasets. It was designed to recover text from
CT scans of ancient, badly damaged manuscripts, but can be applied in many
volumetric analysis applications.

## Installation
### Dependencies
**Required**
* C++17 compiler
* CMake 3.11+
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
* [Doxygen](http://www.doxygen.org/): Required to build
documentation.
* [pybind11](https://github.com/pybind/pybind11): Required to build Python
bindings.

#### (macOS) Homebrew packages
In principle, Homebrew can be used to install all of Volume Cartographer's 
dependencies. However, at the time of this writing, the `vtk` brew package links 
against Qt5 while Volume Cartographer links against Qt6. This will lead to 
linking errors when compiling this library. To use the brew version of VTK, 
you can disable building the VC GUI apps with the `-DVC_BUILD_GUI=OFF` CMake 
flag. Otherwise, you must build VTK from source or follow the instructions for 
[building vc-deps dependencies](#(Optional)-Use-vc-deps-dependencies) below.

### Compilation  
This project is built and installed using the CMake build system. If you have
already installed the dependencies listed above, compilation should be as simple
as:  
```shell
git clone https://github.com/educelab/volume-cartographer.git
cd volume-cartographer
mkdir build
cd build
cmake ..
make
```

Many `volume-cartographer` libraries can be built in parallel, and compilation
times will be improved by running `make -j4`. Alternatively, you can use CMake
to generate [Ninja](https://ninja-build.org/) build system files:  
```shell
cmake -GNinja ..
ninja
```

To install the compiled software and libraries to the `CMAKE_INSTALL_PREFIX`,
run the `install` target:
```shell
make install
```

To generate an installer package, run the `package` target:
```shell
make package
```

This suite is primarily developed and tested on macOS and Debian. Though it
should compile on other Unix/Linux systems and Windows, this has not been
tested. We are accepting contributions to explicitly support other platforms.

#### (Optional) Use vc-deps dependencies
To assist with installing dependencies, we have created the
[vc-deps project](https://github.com/educelab/vc-deps). While this project can
be used on its own to install the dependencies to the system, we also provide
it as a git submodule within `volume-cartographer`. Note that `vc-deps`
**does not** install CMake or Qt.  

To build and link against in-source `vc-deps` libraries, run the following:  
```shell
# Get the source code plus all submodules
git clone --recursive https://github.com/educelab/volume-cartographer.git

# (macOS only)
# brew install boost qt
# brew unlink qt

# Build vc-deps
cd volume-cartographer/vc-deps
mkdir -p build && cd build
# (macOS) Add flag: -DVCDEPS_BUILD_BOOST=OFF
cmake -DCMAKE_BUILD_TYPE=Debug .. 
make -j

# Return to the volume-cartographer directory
cd ../..

# Build volume-cartographer
mkdir -p build && cd build
# (macOS) Add -DCMAKE_PREFIX_PATH flag from the Qt section below
cmake -DCMAKE_BUILD_TYPE=Release -DVC_PREBUILT_LIBS=ON ..
make -j
```

#### Linking against Qt
It might be necessary to point CMake to your Qt installation. For example:
```
# macOS (Apple Silicon), Qt6 installed via Homebrew
cmake -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt/lib/cmake/ ..

# macOS (Intel), Qt6 installed via Homebrew
cmake -DCMAKE_PREFIX_PATH=/usr/local/opt/qt/lib/cmake/ ..

# Ubuntu, Qt6 installed from source
cmake -DCMAKE_PREFIX_PATH=/usr/local/Qt-6.4.2/lib/cmake/ ..
```

### Unit tests
This project is instrumented with unit tests using the Google Test framework.
To enable test compilation, set the `VC_BUILD_TESTS` flag to on:
```shell
cmake -DVC_BUILD_TESTS=ON ..
```

Tests can then be run using CTest or by running the `test` target:
```shell
# Print verbose output with ctest
ctest -V

# Run tests with the test target
make test
```

## Docker
We provide multi-architecture (`amd64` and `arm64`) Docker images in the GitHub
Container Registry. Simply pull our container and Docker will select the
appropriate image for your host platform:

```shell
# Pull the latest image
docker pull ghcr.io/educelab/volume-cartographer

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

To run the GUI tools, you must additionally set up [X11 forwarding from the
container](docs/pages/running-gui-tools.md).

## Documentation
Visit our full library documentation
[here](https://educelab.gitlab.io/volume-cartographer/docs/).

Library documentation is built using Doxygen and can be enabled/disabled by
setting the `VC_BUILD_DOCS` flag. This requires Doxygen and optionally Graphviz.
This option is unavailable if Doxygen is not found. Documentation will be
installed with the `install` target if the `VC_INSTALL_DOCS` flag is enabled.

```shell
cmake -DVC_BUILD_DOCS=ON -DVC_INSTALL_DOCS=ON ..
```

## Python bindings
We currently maintain limited Python binding support through pybind11. They are
a work-in-progress and should not be used in production code.  

Bindings can be built and installed by setting the
`VC_BUILD_PYTHON_BINDINGS` and `VC_INSTALL_PYTHON_BINDINGS` flags:
```shell
cmake -DVC_BUILD_PYTHON_BINDINGS=ON -DVC_INSTALL_PYTHON_BINDINGS=ON ..
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
cmake -DBUILD_SHARED_LIBS=ON ..

# Build static libraries with PIC
cmake -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
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

## References
For more information about the concepts of virtual unwrapping, please see the
following publications:
* William Brent Seales et al. “From damage to discovery via virtual unwrapping:
Reading the scroll from En-Gedi”. In: _Science Advances_ 2.9 (2016).
doi: 10.1126/sciadv.1601247.
url: http://advances.sciencemag.org/content/2/9/e1601247.

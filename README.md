**Volume Cartographer** is a cross-platform C++ library and toolkit for 
virtually unwrapping volumetric datasets. It was designed to recover text from 
CT scans of ancient, badly damaged manuscripts, but can be applied in many 
volumetric analysis applications.

[[_TOC_]]

## Dependencies
**Required**
* C++17 compiler
* CMake 3.11+
* OpenCV 3+
* Insight Toolkit (itk) 4.10+
* Visualization Toolkit (vtk) 7 or 8
* [ACVD Mesh Simplification](https://github.com/csparker247/ACVD) VTK add-on 
library
* [libtiff](https://gitlab.com/libtiff/libtiff) 4.0+
* Eigen3 3.2+
* spdlog 1.4.2+
* Boost Program Options 1.58+: Required if building applications or utilities.
* Qt 5.15+: Required if building applications or utilities.

**Optional**
* Boost Filesystem 1.58+
    - This project will automatically check if the compiler provides 
    `std::filesystem`. If it is not found, then Boost Filesystem is required. 
    This behavior can be controlled with the `VC_USE_BOOSTFS` CMake flag.
* [Doxygen](http://www.doxygen.org/): Required to build 
documentation.
* [pybind11](https://github.com/pybind/pybind11): Required to build Python 
bindings.
* [VCG library](https://github.com/cnr-isti-vclab/vcglib): Required if 
`VC_USE_VCG` is true.

## Compilation  
This project is built and installed using the CMake build system. If you have 
already installed the dependencies listed above, compilation should be as simple 
as:  
```shell
git clone https://gitlab.com/educelab/volume-cartographer.git
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

This suite is primarily developed and tested on macOS and Debian. Though it 
should compile on other Unix/Linux systems and Windows, this has not been 
tested. We are accepting Merge Requests to explicitly support these platforms.

#### (Optional) Use vc-deps dependencies
To assist with installing dependencies, we have created the 
[vc-deps project](https://gitlab.com/educelab/vc-deps). While this project can 
be used on its own to install the dependencies to the system, we also provide 
it as a git submodule within `volume-cartographer`. Note that `vc-deps` 
**does not** install CMake or Qt.  

To build and link against in-source `vc-deps` libraries, run the following:  
```shell
# Get the source code plus all submodules
git clone --recursive https://gitlab.com/educelab/volume-cartographer.git

# Build vc-deps
cd volume-cartographer/vc-deps
mkdir build && cd build
cmake ..
make

# Return to the volume-cartographer directory
cd ../..

# Build volume-cartographer
mkdir build
cd build
cmake -DVC_PREBUILT_LIBS=ON ..
make
```

#### Qt
It might be necessary to point CMake to your Qt installation. For example, 
for Qt5 installed with Homebrew on Mac, 
`-DQt5_DIR=/usr/local/opt/qt/lib/cmake/Qt5` should be added to the `cmake` 
command arguments.

## Installation
To install the compiled software and libraries to the `CMAKE_INSTALL_PREFIX`, 
run the `install` target:
```shell
make install
```

## Packaging

To generate an installer package, run the `package` target:
```shell
make package
```

#### (Optional) Build a deployable macOS installer
To build a deployable macOS installer DMG for macOS 10.13+, use the `vc-deps` 
submodule to build the dependencies as universal libraries:
```shell
# Get the source code plus all submodules
git clone --recursive https://gitlab.com/educelab/volume-cartographer.git

# Setup macOS SDK version
export MACOSX_DEPLOYMENT_TARGET="10.13"

# Build vc-deps
cd volume-cartographer/vc-deps
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_UNIVERSAL_LIBS=ON ..
make
cd ../../

# Setup SDKROOT for volume-cartographer
export SDKROOT=$PWD/vc-deps/build/osx-sdk-prefix/SDKs/MacOSX${MACOSX_DEPLOYMENT_TARGET}.sdk/

# Build volume-cartographer
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DVC_PREBUILT_LIBS=ON ..
make && make package
```

## Testing
Tests are built by default and use the Google Test framework. Tests can be run 
using CTest or by running the `test` target:
```shell
# Print verbose output with ctest
ctest -V

# Run tests with the test target
make test
```

To disable tests, set the `VC_BUILD_TESTS` flag to off:
```shell
cmake -DVC_BUILD_TESTS=OFF ..
```

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

## Python Bindings (WIP)
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

## References
For more information about the concepts of virtual unwrapping, please see the 
following publications:
* William Brent Seales et al. “From damage to discovery via virtual unwrapping: 
Reading the scroll from En-Gedi”. In: _Science Advances_ 2.9 (2016). 
doi: 10.1126/sciadv.1601247. 
url: http://advances.sciencemag.org/content/2/9/e1601247.
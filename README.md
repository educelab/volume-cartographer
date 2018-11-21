Volume Cartographer
===================
A library and toolkit for the virtual unwrapping of volumetric datasets.


### Dependencies  
**Required**  
* C++14 compiler (gcc 4.9+, clang 3.4+ or VS 2015+)
* CMake 3.5+
* OpenCV 3+
* Insight Toolkit (itk) 4.10+
* Visualization Toolkit (vtk) 7 or 8
    * [ACVD Mesh Simplification](https://github.com/valette/ACVD) add-on library
* Boost 1.58+
* Qt 5.7+
* [libtiff](https://gitlab.com/libtiff/libtiff) 4.0+
* Eigen3 3.2+

**Optional**  
* [Doxygen](http://www.doxygen.org/) - For documentation
* [pybind11](https://github.com/pybind/pybind11) - For Python binding support
* [VCG library](https://github.com/cnr-isti-vclab/vcglib) - For VCG's Quadric Edge Collapse Decimation

### Compilation  
The VC suite uses CMake to simplify builds. 
This suite is primarily developed and tested on macOS and Debian. 
It _should_ work on other Unix/Linux systems, but this is currently untested. 
If you have already installed the dependencies listed above, compilation should be as simple as:  
```shell
git clone https://code.cs.uky.edu/seales-research/volume-cartographer.git
cd volume-cartographer
mkdir build
cd build
cmake ..
make
```

Many `volume-cartographer` libraries can be built in parallel, and compilation times will be improved by running `make -j4`.
Alternatively, you can use CMake to generate [Ninja](https://ninja-build.org/) build system files:  
```shell
cmake -GNinja ..
ninja
```


##### (Optional) Use vc-deps dependencies
To assist with installing dependencies, we have created the [vc-deps project](https://code.cs.uky.edu/seales-research/vc-deps).
While this project can be used on its own to install the dependencies to the system, we also provide it as a git submodule within `volume-cartographer`.
Note that `vc-deps` **does not** install CMake or Qt.  

To build and link against in-source `vc-deps` libraries, run the following:  
```shell
# Get the source code plus all submodules
git clone --recursive https://code.cs.uky.edu/seales-research/volume-cartographer.git

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

### Installation
To install the compiled software and libraries to the `CMAKE_INSTALL_PREFIX`, run the `install` target:
```shell
make install
```

### Packaging

To generate an installer package, run the `package` target:
```shell
make package
```

##### (Optional) Build a deployable macOS installer
To build a deployable macOS installer DMG for macOS 10.9+, use the `vc-deps` submodule to build the dependencies as universal libraries:
```shell
# Get the source code plus all submodules
git clone --recursive https://code.cs.uky.edu/seales-research/volume-cartographer.git

# Build vc-deps
cd volume-cartographer/vc-deps
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_UNIVERSAL_LIBS=ON ..
make
cd ../../

# Build volume-cartographer
mkdir build
cd build
export OSX_SDK_VERSION="10.9"
export OSX_SDK_PATH="../vc-deps/build/osx-sdk-prefix/src/osx-sdk/SDKs/MacOSX10.9.sdk/"
cmake -DCMAKE_BUILD_TYPE=Release -DVC_PREBUILT_LIBS=ON -DCMAKE_OSX_DEPLOYMENT_TARGET=$OSX_SDK_VERSION -DCMAKE_OSX_SYSROOT=$OSX_SDK_PATH ..
make && make package
```

### Testing
Tests are built by default and use the Boost.Test framework.
Test can be run using CTest or by running the `test` target:
```shell
make test
```

To disable tests, set the `VC_BUILD_TESTS` flag to off:
```shell
cmake -DVC_BUILD_TESTS=OFF ..
```

If Boost.Test was built with static libraries, you must explicitly tell `volume-cartographer` to link against these static libraries:
```shell
cmake -DBoost_USE_STATIC_LIBS=ON ..
make test
```

### Documentation
Library documentation is built using Doxygen and can be enabled/disabled by setting the `VC_BUILD_DOC` flag.
Documentation will be installed with the `install` target if the `VC_INSTALL_DOC` flag is enabled.
```shell
cmake -DVC_BUILD_DOC=ON -DVC_INSTALL_DOC=ON ..
```

### Python Bindings (WIP)
We currently maintain limited Python binding support through pybind11. 
They are a work-in-progress and should not be used in production code.  

Bindings can be built and installed by setting the `VC_BUILD_PYTHON_BINDINGS` and `VC_INSTALL_PYTHON_BINDINGS` flags:
```shell
cmake -DVC_BUILD_PYTHON_BINDINGS=ON -DVC_INSTALL_PYTHON_BINDINGS=ON ..
```

To use these bindings in Python after installation, import from the `volcart` package:
```python
import volcart.Core as c
import numpy as np

vpkg = c.VolumePkg('/path/to/package.volpkg')
vol = vpkg.volume()
r = vol.reslice(np.array([0,0,0]))
```
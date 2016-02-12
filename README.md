Volume Cartographer
===================

Toolkit to segment and analyze volumetric datasets.

**The Tools**

* Updates coming soon...

**Build Environment**

* C++ Compiler - gcc/clang
* cmake  
* pkg-config
* OpenCV
	* If you want static VC binaries, you must build OpenCV with static libraries: `cmake -DBUILD_SHARED_LIBS=OFF ${OPENCV_SOURCE_PATH}`
* Point Cloud Library (pcl)
    * PCL Requirements: boost, eigen, flann
    * If you want static VC binaries, you must build PCL with static libraries: `cmake -DPCL_SHARED_LIBS=OFF ${PCL_SOURCE_PATH}`
* Insight Toolkit (itk)
* Visualization Toolkit (vtk)
* Boost
* [ACVD Mesh Simplification](https://github.com/valette/ACVD)
* Qt5 
* Meshlab - Recommended for viewing meshes only

**Installation**  
The VC suite uses CMake to simplify builds. This suite is primarly developed and tested on OSX. It _should_ work on other *nix based systems, but this is currently untested. If you have the prerequisites listed above already installed and in your PATH, compilation should be as simple as:
```
git clone https://code.vis.uky.edu/seales-research/volume-cartographer.git
cd volume-cartographer
mkdir build
cd build
cmake ..
make -j4
sudo make install
```  
If you wish to deploy the apps to other machines, prebuild the dependencies statically and run 
`make package` instead of `sudo make install`.  
  
To assist with this task, we have created the [vc-deps project](https://code.vis.uky.edu/seales-research/vc-deps) and have implemented it 
as a git submodule to the main volume-cartographer repository. The only requirement for using this method is that cmake, pkgconfig, and Qt5 be installed first. Using a dynamically linked Qt5 for this process is fine. Here we use the Homebrew provided Qt5, which is currently not placed in the PATH:
```
git clone --recursive https://code.vis.uky.edu/seales-research/volume-cartographer.git
cd volume-cartographer/vc-deps
./build-deps.sh
cd ..
mkdir build
cd build
cmake -DVC_PREBUILT_LIBS=ON -DQt5_DIR=/usr/local/opt/qt5/lib/cmake/Qt5/ ..
make -j4
make package
```
Due to an [on-going issue](https://code.vis.uky.edu/seales-research/volume-cartographer/issues/35), only the GUI applications can be 
deployed in this way.  
  
If building for OSX, you can target OSX 10.9+ by modifying the previous commands to run `./build-deps.sh -universal` instead.
Volume Cartographer
===================

Toolkit to segment and analyze volumetric datasets.

**The Tools**

* VolumePackager - Convert slice data to VC's package format.  
* scalpel - Gradient and structure tensor analysis of volume packages.  
* simulation - Surface segmenter using particle simulation. Uses calculations from scalpel as constraints.  
* texture - Extract texture data from volume set using segmented surface as a guide.  
* utils - Various utilities useful to the segmentation process.  

**Build Environment**

* C++ Compiler - gcc/clang
* cmake  
* pkg-config
* OpenCV compiled with libtiff support
	* If you want static VC binaries, you must build OpenCV with static libraries: `cmake -DBUILD_SHARED_LIBS=OFF ${OPENCV_SOURCE_PATH}`
* Point Cloud Library (pcl)
    * PCL Requirements: boost, eigen, flann
    * If you want static VC binaries, you must build PCL with static libraries: `cmake -DPCL_SHARED_LIBS=OFF ${PCL_SOURCE_PATH}`
* Insight Toolkit (itk)
* Meshlab - Recommended for viewing meshes only

**Installation**
```
git clone https://code.vis.uky.edu/seales-research/volume-cartographer.git
cd volume-cartographer
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
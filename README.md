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
* Point Cloud Library (pcl)
    * PCL Requirements: boost, eigen, flann, vtk
    * Our Requirements: python-vtk, tcl-vtk, libvtk-java, qt
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
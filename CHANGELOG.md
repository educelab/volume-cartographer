Volume Cartographer Changelog
=============================
v2.6.0
------
- apps
    - Fix bug where VC could get hung in an editing mode (!81)
    - Fix rare bug in VC-Texture that could cause app to immediately crash (!79)
- common
    - __New:__ New Point, PointSet, OrderedPointSet data types (!69, !75, !78)
- meshing
    - __New:__ Added Quadric Edge Collapse Decimation (via vcglib) (!70)
    - __New:__ Added OrderedPointSetMesher (!71)
    - Updated testing resources with files that you can actually open (!74)
    - Remove unused PCL functions (!77)
- testing
    - Added tests for ParsingHelpers (!76)
- utils
    - New util to convert PCDs to OrderedPointSet (!72)
- volumepkg
    - Version with integers instead of silly doubles (!80)
- cmake
    - Compiler sanitizer support (!73)
    - Use pragma once for header guards (!68)

v2.5.0
------
This was a huge release. Only notable changes are included below.  
- apps
    - __New:__ Release of the VC-Texture gui app (!14)
    - __New:__ vc_packager++, written in C++ (!9)
    - (VC) Use new LRPS algorithm (!56)
    - (VC) Various bug fixes and UI/UX improvements
- common
    -  __New:__ New common types for all sorts of things: volumes, 
    LRUCaches, Rendering, Textures. EVERYTHING.
- meshing
    - __New:__ CalculateNormals (!54)
    - __New:__ OrderedMeshResampling (!51)
- segmentation
    - __New:__ LocalResliceParticleSimulation (!38)
- testing
    -  __New:__ Unit test framework (!29)
- texturing
    -  __New:__ Several UV map generators: ClothModelingUVMapping, 
    AngleBasedFlattening, LeastSquaresConformalMapping (!44)
- volumepkg
    - Abstract most functionality to the common/types.
- cmake
    - Refactor of the entire cmake build system. We can install stuff now! (!55)
    - Replace picojson with ModernJSON (!49)
    - Migrate GUI apps to Qt5 (!14)

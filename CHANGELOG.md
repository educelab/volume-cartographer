Volume Cartographer Changelog
=============================
v2.7.0
------
This release sees the replacement of PCL's PointCloud datatype with our new
PointSet and OrderedPointSet types! This change allows us to remove the last
traces of PCL from our library, which should improve our build stability. 
**crosses fingers** However, this release is not compatible with version 2
Volume Packages. Please use the `vc_convert_pcd_to_ps` utility to upgrade your
packages to version 3.  
- all
    - Remove PCL usages from the main library (!94). And there was much
    rejoicing!
    - Fix compilation on Linux by adding missing std namespaces to isnan. I mean
    std::isnan (!92)
- apps
    - (VC) Add support for simple linear paths (!90)
    - (VC) Fix crash when navigating while there is no volpkg loaded (!86)
    - (VC) Fix bug where you could get stuck in editing/drawing mode (!81)
    - Learned what C++11 is, so now we test against `nullptr` instead of 
    `NULL` (!89)
- common
    - What? OrderedPointSet is evolving! OrderedPointSet learned `getRow()`, 
    `copyRows()`, and `reset()`! (!96, !97)
    - Remove the `VC_` typedefs and replace them with easy-to-read `volcart::`
    typedefs (!95)
- testing
    - Use boost::split instead of making our own. Seems reasonable. (!88)
- misc
    - Add clang-format style, utilities, and hooks (!87, !93)
    - Fix one of those clang-format utilities (!99)
    - clang-format all of the things! (!98)

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

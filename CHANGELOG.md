Volume Cartographer Changelog
=============================
v2.6.0
------
- common
    - __New:__ New Point and PointSet data types (!69)

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
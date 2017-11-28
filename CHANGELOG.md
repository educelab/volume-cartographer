Volume Cartographer Changelog
=============================
v2.15.0
-------
This is a very small release in order to keep momentum going on the monthly 
releases. The only major change is the addition of better UV map reorientation.

- all
    - (OSX) Fix compiler warnings with the High Sierra-era command line tools 
    (!184)
    - Update `vc-deps` submodule to the most recent version (ace27a06)
- texturing
    - Add UV reorientation so that the dimensions are appropriately aligned 
      along the vertical axis of the UV space (!173)
    - __New:__ Add FlatteningAlgorithmBaseClass for parameterization algorithms 
      (!173)
    - Fixed exception where failure to compute ABF inversion should have fallen 
      back to LSCM (!184)

v2.14.2
-------
Patch release. Fixes bug where VolumePkg v5 wasn't properly registered.

v2.14.1
-------
Patch release. Changes OrderedPointSet::copyRows to exclude the end row index.
This is something the STL does, and something that Sean (skarlage) did in the
first place, but I overruled him like an idiot. YOU WERE RIGHT, SEAN.

v2.14.0
-------
There are a lot of big VolumePkg changes in this release, so you'll want to use
the vc_volpkg_v4to5 utility to upgrade any old packages you may have. In
particular, VolumePkg now supports the new Segmentation and Render objects.
These are meant to provide a canonical interface for dealing with the final
results from each stage of the virtual unwrapping pipeline. They're currently
very basic, but expect them to grow over time as we figure out the best
functionality for each.

- apps
    - Add basic multi-volume support to some of the apps (!172)
    - (vc_packager) Add support for more bit-depths (!174)
    - (vc_projection) Remove the old version and update the new one with program
      options. You can now project any OBJ mesh onto a volume (!175)
- core
    - __New:__ Add new Segmentation class for interacting with segmentation
      results on disk. Updates VolumePkg with the new interface and bumps
      the version to v5 (!176)
    - __New:__ Add new Render class for interacting with render results on disk.
      Also updates VolumePkg with the new interface. This class is fairly empty
      for now, but will be expanding (!178)
    - Remove `vc_defines.hpp`. No more magic includes! (!170)
    - (VolumePkg) Cleanup old volume metadata functions (#170, !178)
- segmentation
    - __New:__ Refactor both LRPS and STPS to use a new common interface,
     ChainSegmentationAlgorithmBaseClass (!177)
    - (STPS) Dramatic rearrangement of this class. Adds Runge-Kutta step
      prediction. Still should not be used in production code (!177)

v2.13.0
-------
This release marks a fairly extensive refactor of the main algorithms in the
Texturing library. The _old_ CompositeTexture and CompositeTextureV2 have been
removed in favor of a _new_ CompositeTexture that no longer relies on the old
texturing utilities. We all wept tears of joy to see it get removed.

VolumePkg also got an upgrade to version 4, and with that upgrade comes basic
support for multi-volumes in one VolumePkg. Old VolumePkg's will need to be
upgraded, so there's also a new utility to do that (vc_volpkg_v3to4).

Also, a shout-out to Hannah Hatch (hjha225) for finishing up Doxygen
documentation on all of the main VC libraries. It was no small task, and we're
all in debt to Hannah's incredible patience.

- core
    - Complete Doxygen documentation (!158)
    - (VolumePkg) Move to the volcart namespace and add multi-volume support.
     Upgrades VolumePkg to v4 (!161)
- texturing
    - Complete Doxygen documentation (!167)
    - __New:__ Texture algorithms galore! CompositeTexture, IntersectionTexture,
     IntegralTexture, and LayerTexture (!168)

v2.12.0
-------
- all
    - Add CI testing for dynamic linking (!163)
- core
    - (OBJReader) Fix relative image path bug (!164)
    - (OBJWriter) Add compression to texture image output (7a3e931c)
    - (VolumePkg) Add shared pointer typedefs (!160)
- texturing
    - Scale ABF generated mesh to same surface area as input mesh (!162)

v2.11.0
-------
- apps
    - (VC.app) Actually use the reslice window width parameter for LRPS (!156)
    - (VC.app) "Next Slice" button skips by 10 slices when holding the Shift
    key (!157)
    - (vc_packager) Add option to flip slice images (!156)
    - (vc_segment) Add option for reslice window size. Fix bugs in LRPS
    k2 parameter and the calculation of the starting index (!156)
- segmentation
    - (LRPS) Add option for changing reslice window size (!156)
    - (LRPS) Interpolate eigenvalues at floating point positions when estimating
    surface normals (!156)
    - (LRPS) Rework debug drawing to produce clearer images (!156)
- texturing
    - Remove unnecessary origin flips in UV generation (!156)

v2.10.0
-------
- all
    - Change all C++ header file extensions to `.hpp` (!135)
    - Fix problem for external projects where headers couldn't be found. From
     now on, all includes should have the `vc/` subdirectory. e.g. `#include
      <vc/core/vc_defines.hpp>` (!153)
    - Update Modern JSON to version 2.1.0 (!146)
    - Add a blacklist to clang-format (!147)
    - Convert inline To-Do's with Gitlab issues (!145)
    - Add CMake VC_USE_ALL option to enable all optional dependencies. This
    should be used with care and really only as an option for development (!142)
- apps
    - Fix bad cast of program options to enumerated values (!141)
- core
    - __New:__ OBJReader: Read a textured OBJ file! Oh the possibilities! (!116)
    - Fix bug in OBJWriter where integer types were getting written as floats.
    Also add conditionally writing vertex normals on whether or not the mesh
    actually has that information. It's kind of amazing that this didn't cause
    problems before now (!152)
    - Simplify HalfEdgeMesh subtypes into structs (!151)
    - Standardize OrderedPointSet on the y,x operator ordering using by OpenCV.
    This will just save us a lot of confusion (!143)
    - PerPixelMap refactored to use OrderedPointSet as its base type (!137)
- docs
    - Add support for building Doxygen documentation through CMake and the
    build system (!144)
    - Add Contributors log for the main VC team (!139)
- external
    - Move GetMemorySize to external lib (!136)
- meshing
    - Remove deprecated ITK feature from ScaleMesh (!148)
- segmentation
    - Add Doxygen documentation (!125)
- utils
    - Remove PCL point cloud converter. And just like that, the last vestiges
    of PCL's tyranny are gone (!140)

v2.9.0
------
This release constitutes a major refactor of the entire API. Any previously
opened branches will need to be merged or rebased. Most things are
exactly where they were before, only now the functions and files are named
consistently (expect lots of case and underscore changes). We'll be updating the
style guides soon to reflect these changes.

We also updated to require OpenCV 3+. This will probably cause headaches in the
short term as it can cause conflicts if installed alongside OpenCV 2, but the
benefits should outweigh the costs moving forward.
- all
    - Improvements to the CI system (!124, !121)
    - Update everything to use OpenCV 3+ (!131)
    - Add ability to import VC libraries into external CMake projects using
    find_package (!127) Currently requires the external project to find_package
    dependencies as well. We hope to improve support for this in the future.
    - Enable tons of build warnings and fix most of them! (!126)
    - Huge refactor of everything to enforce code style, naming rules, and
    clang-tidy rules (!130, !133)
- apps
    - Fix bug in vc_metaedit where numerical data would get saved as a string
    (!129)
- core
    - Newly refactored PLYReader that can better handle unknown elements and
    attributes (!110)
    - Replace all vc::Point usages with cv::Vec (!128) We didn't really need it.
- texturing
    - __New:__ PPMGenerator: Generates a rastered UV Map for doing rendering and
    provenance chain lookups (!117)

v2.8.0
------
- all
    - Project now uses C++14 (!112)
- apps
    - (VC-Texture) Warn the user if changing segmentations/VolPkgs will result
    in data loss. We won't let you save your changes, but we definitely won't
    let you lose them, either (!106)
    - (VC-Texture) Disable menu options while generating a texture image (!102)
- core
    - Rename vc_common library to vc_core (!119)
    - Fix bug that wouldn't let you overwrite values in a UV Map (!108)
    - Various little bug fixes related to texturing tasks (!111)
    - VolumePkg now only depends on vc_core (!113, !115)
    - (VolumePkg) Why did we have an initialize() function in the first place?
    (!114)
- docs
    - Added Doxygen documentation framework and examples (!100)
- meshing
    - Add Doxygen documentation (!109)
- texturing
    - Improvements (not fixes, unfortunately) to ABF parameterization (!107)
- volumepkg
    - Move VolumePkg to vc_core and remove vc_volumepkg library (!118). Goodbye
    old friend.
    - Add Doxygen documentation (!100)

v2.7.1
------
Patch: Update release dependencies to latest versions.

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

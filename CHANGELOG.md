Volume Cartographer Changelog
=============================
v2.19.0
-------
This release contains a major refactor of the `Volume` class, which brings it 
in line with the `DiskBasedObject` interface that is intended for the major
`VolumePkg` sub-objects. It should functionally be the same with the exception 
of the `StructureTensor` methods, which have become free functions in
`volcart::Core`. You can now find them in `vc/core/math/StructureTensor.hpp`.

- all
    - Update CI and submodule settings to reflect code server migration (!223)
- apps
    - (layers_from_ppm) New option to save a PPM from in the layers volume (!218)
- core
    - __New:__ NeighborhoodGenerators: Common interface for requesting ROIs 
    from a Volume (!221)
    - (Volume) Cleanup interface and derive from DiskBasedObject (!222)
    - Moved StructureTensor methods to `vc/core/math/StructureTensor.hpp` (!222)
    - (VolumePkg) Add v6 metadata
- testing
    - Upgrade `Testing.volpkg` to v6 (499b7ea0)
- texturing
    - Refactor all texturing algorithms to use NeighborhoodGenerators interface
     (!221)
    - Small performance fixes post-refactor (!223)
- utils
    - __New:__ `vc_volpkg_upgrade`: Single utility to upgrade a VolumePkg to the 
    latest version (!222)
    - __New:__ `vc_transform_mesh`: New utility to apply affine transforms to a 
    mesh (!223)

v2.18.1
-------
Patch release to fix CI builds. Success deploy builds are now uploaded to
Google Drive.
- all
    - (CI) Disable all vcglib builds
- apps
    - (packager) Add basic support for slice formats other than TIFFs. Add
    z-flip/invert slice ordering option. (!217)
- utils
    - (invertcloud) Fix bug where this didn't actually work anymore (!219)

v2.18.0
-------
This release is long overdue, but contains a lot of very important changes, 
particularly to texturing. First, a major bug in `AngleBasedFlattening` has 
been fixed (!210), resulting in dramatically improved computation times for 
large meshes. Second, the texturing algorithms have been reworked to optimize 
neighborhood requests from the `Volume` (!206, !212), decreasing the cache 
thrashing that would occur when the UV maps were axis-aligned. Finally, there 
are a lot of new options and flags in the apps related to UV maps and the 
integral texture method.

- all
    - Update `find_package` calls with the actual minimum versions of 
    dependencies (!204)
    - Update to latest `vc-deps` which uses CMake for building (!213)
    - Rename CMake option `VC_USE_ALL` -> `VC_USE_OPTIONAL` (!215)
- apps
    - Improve reporting when there's an exception in the cmd line apps (!198)
    - (mesher) Fix bug where mesh was not being written. Add vertex-only mode. 
    (!200)
    - (render) Add mesh smoothing option (!199)
    - (render\[_from_ppm\]) Add new option `cache-memory-limit` to limit the 
    Volume's memory usage. (!202)
    - (render) Load mesh as input and new options around mesh resampling (!202)
    - (render) Add options for UV rotation and plotting (!202)
    - (render) Add option to reuse UV map from input mesh if one is available 
    (!207)
    - Fix bug in various apps for auto-radius calculation (!207)
    - (projection) Add support for projecting multiple meshes (!207)
- core
    - __New:__ `TIFFIO`: Method for writing floating-point tiff images (!202)
    - __New:__ `FileExtensionFilter`: Method to filter a path by a list of 
    recognized extensions (!202)
    - __New:__ `MemorySizeStringParser`: Convert strings such as "6TB" to the 
    corresponding integer amount of bytes (!202)
    - (UVMap) Add `Rotate` method (!202)
    - (UVMap) Move `drawUV` -> `Plot` (!202)
    - (UVMap) Add `Flip` method (!208)
- __New:__ experimental library
    - Move `texturing::ClothModelingUVMapping` and `meshing::ITK2Bullet` to 
    `experimental` library (!215) Note that this also moves the classes into a 
    new namespace, `experimental::texturing` and `experimental::meshing` 
    respectively.
- texturing
    - (FlatteningAlgorithmBaseClass) Fix bug where reorientation wasn't 
    rotating around the center of the UV map (!201)
    - (IntegralTexture) Add new filtering and weighting options (!202):
        - Exponential Difference weighting: Weight neighborhood values by their 
        difference from a base value (mean, mode, or user-provided), taken 
        to an exponent
        - Clamp values to maximum intensity value
    - (IntegralTexture) Return floating-point image. Add corresponding option 
    to apps (!209)
    - (ABF) Fix bug in `HalfEdgeMesh` construction that were making ABF much 
    slower than it should have been (!210)
    - Dramatically speed up all texturing algorithms when the UV map is not 
    axis-aligned (!206, !212)
- utils
    - __New:__ `vc_ppm_to_pointset` Create a point set from a PPM's mappings 
    (!212)

v2.17.0
-------
- apps
    - (packager) Add ability to add Volumes to existing Volume Packages (!196)
    - __New:__ `vc_render_from_ppm`: App that renders texture images using a 
    precomputed PPM (!195)
- core
    - Fix PPM mask loading bug (!194)
    - Add Associated Volume metadata to Segmentations (!195). This feature links 
    a Segmentation to the Volume used as its originating frame of reference, an 
    essential part of our digital provenance chain. Apps have also been updated 
    to make use of this information when possible.
- external
    - Update to Modern JSON v3 (!193)

v2.16.0
-------
This release holds an exciting addition for our Tensorflow efforts: Python
bindings for some of the essential classes in the Core library! Besides being
able to load Volume Packages, we think using the arbitrary reslice and subvolume
extraction features of the Volume class will be particularly useful for machine
learning on volumetric datasets. The bindings use the excellent
[pybind11](https://github.com/pybind/pybind11) library, which made coding up
the interface way easier than we thought it would be. Expanding the bindings
to all classes and all project libraries is planned, but probably won't be
explored extensively until after the 3.0 release.

This release also (finally) includes proper component installation via CMake.
You can find precompiled static libraries for macOS in this release's DMG.

- all
    - Proper component installation via CMake (!189)
    - (CI) Start doing weekly pre-release builds. Adds the CMake flag
    `VC_VERSION_DATESTAMP`, which appends the configure time and date to the
    project version number (43acbe10, b0370b05)
    - Fix some of the weirdness around warnings with different compilers
    (1bdb9a07)
- apps
    - (VC-Texture) Add support for Intersection and Integral texturing (!190)
    - (VC-Texture) Fix a bug where attempting to texture a segmentation with
    only a single row of points resulted in an endless "processing" loop. (!190)
    - (VC-Texture) Majorly cleanup this app, because I couldn't stand the
    clutter... (!190)
- core
    - __New:__ Python bindings for some of the essential classes. These can be
    built by enabling the `VC_BUILD_PYTHON_BINDINGS` CMake flag. If
    `VC_INSTALL_PYTHON_BINDINGS` is enabled, a PyPI egg will be installed to your
    current Python environment. See the
    [pybind11 documentation](http://pybind11.readthedocs.io/en/stable/compiling.html#configuration-variables)
    to control the version of Python for which the bindings are built. (!188)
    - __New:__ Spiral shape generator! It looks like a scroll! Hooray! (!191)

v2.15.1
-------
Patch release. Fixed vc-deps dependency issues. Sets up weekly dev build.

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

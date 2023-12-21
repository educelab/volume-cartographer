# Volume Cartographer Apps and Utilities

An exhaustive list of the applicaitons and utilities provided by Volume 
Cartographer. Most programs can be run with `--help` for more usage details.

## VC
The primary GUI interface for performing segmentation with Volume Cartographer.

**Installation note:**
On macOS, this program is compiled into `VC.app` and can be run by 
double-clicking the app bundle. When installing with Homebrew, `VC.app` is 
placed in `/Applications/`. On all other platforms, the `VC` executable is 
installed to the system prefix and can be launched from the command line.

## CannySegment
A segmentation tool which uses [canny edge detection](https://en.wikipedia.org/wiki/Canny_edge_detector)
to identify surface points. More details on working with `CannySegment` are 
available [here](https://github.com/educelab/ink-id/blob/develop/docs/data-processing-workflow.md#exposed-layers).

**Installation note:**
This program is primarily a command line tool, but is compiled into 
`CannySegment.app` on macOS because it contains optional GUI components. It can 
be launched by running the packaged executable:
```shell
./CannySegment.app/Contents/MacOS/CannySegment
```

When installing [via Homebrew](../../README.md#using-homebrew), the packaged
executable is symlinked to `$(brew --prefix)/bin/vc_canny_segment` and can be 
run directly. If installing from source, you can create your own symlink into 
the system search path:

```shell
# Installing from source on macOS only
# ln -s ./CannySegment.app/Contents/MacOS/CannySegment /usr/local/bin/vc_canny_segment
vc_canny_segment
```

## MeshProject
Visualization tool that draws the intersection of a mesh/meshes on the slices 
of a volume.

**Installation note:**
This program is primarily a command line tool, but is compiled into
`MeshProject.app` on macOS because it contains optional GUI components. It can
be launched by running the packaged executable:
```shell
./MeshProject.app/Contents/MacOS/MeshProject
```

When installing [via Homebrew](../../README.md#using-homebrew), the packaged
executable is symlinked to `$(brew --prefix)/bin/vc_projection` and can be run
directly. If installing from source, you can create your own symlink into
the system search path:

```shell
# Installing from source on macOS only
# ln -s ./MeshProject.app/Contents/MacOS/MeshProject /usr/local/bin/vc_projection
vc_projection
```

##  vc_packager
Program for creating and adding volumes to a Volume Package (`.volpkg`) 
directory. Volume packages can be thought of as a project directory for the 
Volume Cartographer suite.

```shell
# Make a new .volpkg
vc_packager -v my-project.volpkg -s path/to/first-volume/ --material-thickness 600

# Add a volume to an existing .volpkg
vc_packager -v my-project.volpkg -s path/to/second-volume/
```

## vc_volpkg_explorer
Displays the contents of a Volume Package (`.volpkg`).

```shell
$ vc_volpkg_explorer -v my-project.volpkg 

 --- VolumePkg ---
Name: My unwrapping project
Material Thickness: 1000um

 --- Volumes ---
[20230315130124] Campfire Scroll, 560x560x477, 104um/voxel

 --- Segmentations ---
[20230315130225] 20230315130225, associated volume: 20230315130124

 --- Renders ---
```

## vc_render
The primary tool for rendering (meshing + flattening + texturing) a 
virtually unwrapped result from a segmentation.

```shell
# Use vc_volpkg_explorer to get the segmentation ID for -s
vc_render -v my-project.volpkg -s 20230315130225 -o first-result.obj
```

## vc_render_from_ppm, vc_layers_from_ppm
Special version of `vc_render` which use a pre-generated per-pixel map 
(PPM) rather than meshing and flattening a segmentation. Saves time when 
you want to compare renders generated using different texturing parameters.

```shell
# Generate the PPM
vc_render -v my-project.volpkg -s 20230315130225 -o params-1.tif --output-ppm seg-map.ppm

# Reuse the PPM to generate a new texture with different parameters
vc_render_from_ppm -v my-project.volpkg -p seg-map.ppm -o params-2.tif --filter 3
```

## vc_segment
A command line tool for running segmentation algorithms. To get started, start 
a new segmentation in the main `VC` GUI, then use this tool to propagate the 
result to new slices. Includes the Thinned Flood Fill algorithm, which is not 
yet available in the GUI.

## vc_convert_pointset
Convert a Volume Cartographer point cloud file (`.vcps`) to a mesh file 
(PLY/OBJ). Does not perform triangulation.

## vc_mesher
Triangulate (i.e. mesh) a Volume Cartographer point cloud file (`.vcps`). The 
input point cloud should be *ordered* (i.e. stored as a 2D matrix), the default 
type created by `VC`.

## vc_retexture_mesh
Given a textured mesh, replace its texture with in an alternate texture image. 
Useful if you used `vc_render_from_ppm` to generate a texture that you want 
to visualize on a 3D mesh.

## vc_flatten_mesh
Flattens a triangulated mesh using one of our parameterization algorithms. The 
input mesh should have a surface boundary, be manifold, and have consistent 
winding order.

## vc_color_map
Apply a color map to an image file, usually a texture image. Useful when looking
for content that may be difficult to see in grayscale.

## vc_merge_pointset
Merge multiple point cloud (`.vcps`) files into a single point cloud. Useful 
when combining multiple segmentations into a single surface.

## vc_visualize_graph
The processing done by `vc_render` is executed by our 
[smgl](https://github.com/educelab/smgl) graph processing library. This utility 
allows you to visualize the render graph.

```shell
# Create a render in the volpkg by rendering a segmentation
# The Render ID is reported on the command line
vc_render -v my-project.volpkg -s 20230315130225
# [volcart] [info] Created new Render graph in VolPkg: 20230316104344

# Save render graph to a graphviz dot file
vc_visualize_graph -v my-project.volpkg -r 20230316104344 -o graph.gv

# Render image with graphviz
dot -Tpng graph.gv > graph.png
```

## vc_area
Calculates the surface area of a segmentation in physical units.

## vc_transform_uv
Rotates and/or flips a mesh's UV map and texture image. Useful if text or
content in the texture image appears rotated or flipped with respect to the
image axes.

## vc_transform_mesh
Apply various linear transforms to a mesh. Primarily useful for visualization
purposes.

## vc_volpkg_upgrade
We occasionally upgrade the Volume Package (`.volpkg`) file format to support 
new features. This tool upgrades existing volume packages to the new format.

## vc_repair_pointsets
A [bug](https://github.com/educelab/volume-cartographer/issues/24) in certain
versions of VC resulted in ordered `.vcps` files where the z-values of points 
were off by a few slices. This utility repairs these files by assigning new 
z-indices to all points relative to the z-index of the first row:

| row | old z | new z |
|-----|-------|-------|
| 0   | 400   | 400   |
| 1   | 400   | 401   |
| 2   | 401   | 402   |
| ... | ...   | ...   |
| 9   | 408   | 409   |
| 10  | 410   | 410   |
| 11  | 410   | 411   |

The utility can be run from the command line:
```shell
# See what will be modified
vc_repair_pointsets -i pointset.vcps --verbose

# Repair the pointset and save the result to a new file
vc_repair_pointsets -i pointset.vcps -o repaired.vcps

# Backup and repair the pointset
cp pointset.vcps pointset.vcps.orig
vc_repair_pointsets -i pointset.vcps -o pointset.vcps
```

## vc_visualize_ppm
This file generates visualizations of the positions and normals saved in PPM 
files. Files are saved as 32-bit, RGB TIFs:
```shell
# Generate surface_maps_normal.tif and surface_maps_pos.tif
vc_visualize_ppm --volpkg my-project.volpkg --ppm surface.ppm -o surface_maps_
```

## Other utilities
These are extra utilities available in the Volume Cartographer build directory 
when building from source. They are largely developer tools.

```
vc_add_alignment_markers
vc_cmap_bar
vc_convert_mask
vc_flip_mesh
vc_generate_ppm
vc_image_stats
vc_invert_cloud
vc_metaedit
vc_ppm_tool
vc_project_mesh
vc_seg_to_pointmask
vc_uv2mesh
vc_volume_bump
vc_volume_client
vc_volume_server
```